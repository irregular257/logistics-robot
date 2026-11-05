"""K230 / CanMV YOLO image collector. Run this file in CanMV IDE.

Standalone script: does not import main.py or send motor commands.
Channel 0 provides hardware video preview; channel 1 saves clean RGB photos.
Images are saved without boxes, text, resizing, or detection filters.
"""
import gc
import json
import os
import sys
import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

# ---------- 修改这里即可 ----------
SAVE_ROOT = "/data/yolo_capture"  # 照片保存到 data 目录
CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480
INTERVAL_MS = 500             # 每两秒拍一张；按保存完成时间计时
MAX_PHOTOS = 200           # 本次拍摄上限；0 表示一直拍，手动停止
WARMUP_MS = 3000           # 先预览三秒，等待曝光和白平衡稳定
JPEG_QUALITY = 95
HMIRROR = False            # 与实际部署方向一致
VFLIP = False
IDE_PREVIEW = True         # 不连接 IDE 独立运行时可以改为 False
PREVIEW_FPS = 30           # 预览独立于拍照间隔；实际帧率取决于连接和固件
# ---------------------------------

CAPTURE_CHANNEL = 1


def new_batch(root):
    root = root.rstrip("/")
    if not root.startswith("/") or not root:
        raise ValueError("SAVE_ROOT must be an absolute subdirectory path")
    # 不创建挂载目录本身：/data 或 /sdcard 必须已存在且可访问。
    # 保存路径使用已存在目录下的一层子目录，例如 /data/yolo_capture。
    parent = root.rsplit("/", 1)[0] or "/"
    os.listdir(parent)
    try:
        os.mkdir(root)
    except OSError as exc:
        if exc.args[0] != 17:  # EEXIST；其他错误（如只读）必须上报
            raise
        os.listdir(root)      # 确认不是同名普通文件
    for number in range(1, 1000000):
        name = "batch_%04d" % number
        path = root + "/" + name
        try:
            os.mkdir(path)    # 新批次，不覆盖已有文件
            return path, name
        except OSError as exc:
            if exc.args[0] != 17:
                raise
    raise RuntimeError("Too many capture batches")


def storage_available(path):
    # 以实际挂载分区为准；改目录本身不会增加可用容量。
    if not hasattr(os, "statvfs"):
        return None
    try:
        info = os.statvfs(path)
        block_size = info[1] or info[0]
        available = block_size * info[4]
        return available
    except OSError as exc:
        print("Cannot read storage free space:", exc)
        return None


def check_space(path):
    available = storage_available(path)
    # 留出一张照片及文件系统开销；未知容量时仍由保存接口报告写入错误。
    reserve = max(2 * 1048576, CAPTURE_WIDTH * CAPTURE_HEIGHT * 2 + 1048576)
    if available is not None and available < reserve:
        raise OSError(28, "Storage nearly full; copy photos to the PC first")


def save_jpeg(img, folder, batch, number):
    final_path = "%s/%s_%06d.jpg" % (folder, batch, number)
    temporary = folder + "/.pending.jpg"
    # 使用 JPG 扩展名让 CanMV 编码 JPEG，不向训练照片绘制框或文字。
    try:
        img.save(temporary, quality=JPEG_QUALITY)
        # 防止固件接口变化把原始像素数据误当作 JPG。
        with open(temporary, "rb") as stream:
            start = stream.read(2)
            stream.seek(-2, 2)
            end = stream.read(2)
        if start != b"\xff\xd8" or end != b"\xff\xd9":
            raise RuntimeError("Saved file is not a complete JPEG")
        size = os.stat(temporary)[6]
        os.rename(temporary, final_path)
        if hasattr(os, "sync"):
            os.sync()
        return final_path, size
    finally:
        try:
            os.remove(temporary)
        except OSError as exc:
            if exc.args[0] != 2:  # ENOENT：成功重命名后临时文件已不存在
                print("Temporary file cleanup failed:", exc)


def cleanup(sensor, display_ready, media_ready):
    # 各阶段独立清理，停止相机失败也继续释放显示和媒体资源。
    if sensor is not None:
        try:
            sensor.stop()
        except Exception as exc:
            print("Camera cleanup:", exc)
    if display_ready:
        try:
            Display.deinit()
        except Exception as exc:
            print("Display cleanup:", exc)
    if media_ready:
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)
        except Exception as exc:
            print("Media shutdown delay:", exc)
        finally:
            try:
                MediaManager.deinit()
            except Exception as exc:
                print("Media cleanup:", exc)


def main():
    if INTERVAL_MS < 100 or MAX_PHOTOS < 0 or WARMUP_MS < 0 or PREVIEW_FPS <= 0:
        raise ValueError("Invalid capture timing/count")
    if not 1 <= JPEG_QUALITY <= 100:
        raise ValueError("JPEG_QUALITY must be in 1..100")
    if CAPTURE_WIDTH <= 0 or CAPTURE_WIDTH % 16 or CAPTURE_HEIGHT <= 0:
        raise ValueError("Width must be positive and aligned to 16 pixels")

    sensor = None
    display_ready = media_ready = False
    saved = 0
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        folder, batch = new_batch(SAVE_ROOT)
        print("Save folder:", folder)
        available = storage_available(folder)
        if available is not None:
            print("Storage available: %.1f MiB" % (available / 1048576))
        check_space(folder)

        # 自动识别当前相机，不硬编码 GC2145 或 GC2093。
        sensor = Sensor(width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT)
        sensor.reset()
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
        # 预览使用 YUV 视频层，不反复复制 RGB 图或在图上绘字。
        # 两个通道均在 run() 前配置，拍摄期间不改变相机分辨率。
        sensor.set_framesize(width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT, chn=0)
        sensor.set_pixformat(Sensor.YUV420SP, chn=0)
        sensor.set_framesize(width=CAPTURE_WIDTH, height=CAPTURE_HEIGHT,
                             chn=CAPTURE_CHANNEL)
        sensor.set_pixformat(Sensor.RGB565, chn=CAPTURE_CHANNEL)
        if IDE_PREVIEW:
            Display.bind_layer(**sensor.bind_info(), layer=Display.LAYER_VIDEO1)
            Display.init(Display.VIRT, width=CAPTURE_WIDTH,
                         height=CAPTURE_HEIGHT, fps=PREVIEW_FPS, to_ide=True)
            display_ready = True
        MediaManager.init()
        media_ready = True
        sensor.run()
        camera_type = str(sensor.get_type())
        print("Camera:", camera_type)
        print("Preview warmup; first photo after %d ms" % WARMUP_MS)

        with open(folder + "/session_info.json", "w") as stream:
            json.dump({"batch": batch, "firmware": str(os.uname()),
                       "camera": camera_type, "width": CAPTURE_WIDTH,
                       "height": CAPTURE_HEIGHT, "hmirror": HMIRROR,
                       "vflip": VFLIP, "jpeg_quality": JPEG_QUALITY,
                       "interval_ms": INTERVAL_MS,
                       "max_photos": MAX_PHOTOS}, stream)

        start = time.ticks_ms()
        last_saved = None
        while MAX_PHOTOS == 0 or saved < MAX_PHOTOS:
            os.exitpoint()  # 响应 IDE 停止按钮 / Ctrl+C
            now = time.ticks_ms()
            warmed_up = time.ticks_diff(now, start) >= WARMUP_MS
            due = last_saved is None or time.ticks_diff(now, last_saved) >= INTERVAL_MS
            if warmed_up and due:
                check_space(folder)
                capture_start = time.ticks_ms()
                img = sensor.snapshot(chn=CAPTURE_CHANNEL)
                if img is None:
                    raise RuntimeError("Camera returned no image")
                try:
                    if img.width() != CAPTURE_WIDTH or img.height() != CAPTURE_HEIGHT:
                        raise RuntimeError("Actual image size differs from settings")
                    path, size = save_jpeg(img, folder, batch, saved + 1)
                finally:
                    del img
                    gc.collect()
                saved += 1
                last_saved = time.ticks_ms()
                print("SAVED %d: %s (%d bytes, %d ms)" %
                      (saved, path, size, time.ticks_diff(last_saved, capture_start)))
            # 相机到显示的绑定持续输出画面；此处只轮询拍照时间。
            time.sleep_ms(20)
        print("Capture complete:", saved, "photos")
    except KeyboardInterrupt:
        print("Capture stopped:", saved, "photos saved")
    except Exception as exc:
        print("Capture failed. Check camera, storage free space/write access and firmware.")
        sys.print_exception(exc)
        raise
    finally:
        cleanup(sensor, display_ready, media_ready)


if __name__ == "__main__":
    main()
