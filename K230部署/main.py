import ustruct, time
from k230_hal import sensor, create_uart, set_light, show_image, shutdown, preview_image, draw_text
from vision_colors import COLOR_PROFILES, detect_materials
from ulab import numpy as np
from vision_tracking import ColorTracker, tracking_packet, target_tracking_packet, TargetCommandParser
import yolo_config
from vision_yolo import YoloDetector, tracking_candidates, draw_detections

HOUGH_THRESHOLD = 2000
MIN_RADIUS = 10
MAX_RADIUS = 50
TS = 1/60

clock = time.clock()

class KalmanFilter:
    def __init__(self, initial_state):
        self.A = np.array([
            [1, 0, 0, 0, TS, 0],
            [0, 1, 0, 0, 0, TS],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        self.C = np.eye(6)
        self.Q = np.diag([1e-6]*6)
        self.R = np.diag([1e-6]*6)
        self.x_hat = initial_state
        self.p = np.diag([10]*6)

    def update(self, Z):
        x_hat_minus = np.dot(self.A, self.x_hat)
        p_minus = np.dot(self.A, np.dot(self.p, self.A.T)) + self.Q
        S = np.dot(self.C, np.dot(p_minus, self.C.T)) + self.R
        S_inv = np.linalg.inv(S + 1e-4*np.eye(6))
        K = np.dot(np.dot(p_minus, self.C.T), S_inv)
        innovation = Z - np.dot(self.C, x_hat_minus)
        self.x_hat = x_hat_minus + np.dot(K, innovation)
        self.p = np.dot((np.eye(6) - np.dot(K, self.C)), p_minus)
        return self.x_hat

kf = KalmanFilter(np.array([80, 60, 30, 30, 2, 2]))

# K230 light pin is optional; configure it in k230_config.py.

Block_Roi = [70, 30, 160, 160]
data = [0, 0]

# 调试模式：设置为True可以打印检测到的色块LAB值，用于微调阈值
DEBUG_MODE = False  # 改为True开启调试，False关闭

# 强光环境优化的色环阈值 - 根据实测值调整
red_circle_threshold = (15, 50, 20, 60, 5, 50)          # 红色环 - 实测L=26,A=35,B=26
green_circle_threshold = (8, 75, -60, -11, 6, 70)     # 绿色环 - 实测L=62,A=-52,B=45
blue_circle_threshold = (25, 65, -10, 35, -65, -15)     # 蓝色环 - 实测L=43,A=12,B=-39

# 优化的颜色阈值 (L, A, B) - 根据实测值调整
# 格式: (L_min, L_max, A_min, A_max, B_min, B_max)
thresholds = [
    (19, 73, 6, 51, 2, 78),        # 红色 - 实测L=26,A=35,B=26
    (25, 85, -66, -15, -42, 64),     # 绿色 - 适度收紧,避免浅绿色
    (15, 89, -20, 35, -68, -24)       # 蓝色 - 大幅放宽范围
]

uart = None  # Created after camera startup in the guarded main loop.

# 缓存命令5检测到的坐标，供命令7使用，保证坐标一致
cached_ring_x = 0
cached_ring_y = 0
cached_ring_valid = False

def send_data_Colorround(Color_task):
    data = ustruct.pack("<BBB", 0x03, int(Color_task), 0xFE)
    uart.write(data)

def send_data_Colorround_cmd21_23(Color_task):
    data = ustruct.pack("<BBB", 0x04, int(Color_task), 0xFE)
    uart.write(data)

def send_data_adjustment_Colorround(x_dir, y_dir):
    data = ustruct.pack("<BBBB", 0x01, int(x_dir), int(y_dir), 0xFE)
    uart.write(data)

def send(y_distance, x_distance):
    data = ustruct.pack("<BBBB", 0xff, int(y_distance), int(x_distance), 0xFE)
    uart.write(data)

def send1(y_distance, x_distance, code):
    data = ustruct.pack("<BBBBB", 0xff, int(y_distance), int(x_distance), int(code), 0xFE)
    uart.write(data)

def send2(y_distance, x_distance, code, code1):
    data = ustruct.pack("<BBBBBB", 0xFA, int(y_distance), int(x_distance), int(code), int(code1), 0xFE)
    uart.write(data)

# Tracking commands: 31=red, 32=green, 33=blue, 34=any; 30=stop.
# The original detection commands keep their original packet formats.
tracker = ColorTracker(ticks_diff=time.ticks_diff)
command_parser = TargetCommandParser(ticks_diff=time.ticks_diff)
target_request = None  # (request token, color, shape); mode 35 uses framed tasks.
tracking_sequence = 0
tracking_last_send = None
current_frame_size = sensor.QVGA
TRACK_SEND_PERIOD_MS = 50
TRACK_PIXELS_MIN = 200
TRACK_AREA_MIN = 200
yolo = None


def set_frame_size(size):
    global current_frame_size
    if current_frame_size != size:
        sensor.set_framesize(size)
        current_frame_size = size


def run_tracking(img, command):
    global tracking_sequence, tracking_last_send
    now = time.ticks_ms()
    candidates = []
    colors = (1, 2, 3) if command == 34 else (command - 30,)
    # Separate color passes prevent a merged red/green object changing identity.
    for color in colors:
        blobs = img.find_blobs([thresholds[color - 1]],
                              pixels_threshold=TRACK_PIXELS_MIN,
                              area_threshold=TRACK_AREA_MIN, merge=True, margin=5)
        for blob in blobs:
            candidates.append((blob.cx(), blob.cy(), blob.w(), blob.h(), blob.pixels(), color,
                               blob.rect()))
    result = tracker.update(candidates, now)
    label = "Y=%dpx dY=%dpx" % (result[3], 120 - result[3]) if result[0] else "Target lost"
    draw_text(2, 2, 16, label, color=(255, 255, 255))
    if result[0]:
        _, color, cx, cy, width, height = result
        img.draw_rectangle((max(0, cx - width // 2), max(0, cy - height // 2), width, height),
                           color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(255, 0, 0))
    img.draw_cross(160, 120, color=(255, 255, 255))
    if tracking_last_send is None or time.ticks_diff(now, tracking_last_send) >= TRACK_SEND_PERIOD_MS:
        uart.write(tracking_packet(tracking_sequence, result))
        tracking_sequence = (tracking_sequence + 1) & 0xFF
        tracking_last_send = now


def run_yolo_tracking(img, detections, canvas, command):
    global tracking_sequence, tracking_last_send
    now = time.ticks_ms()
    wanted = (target_request[1],) if target_request else ((1, 2, 3) if command == 34 else (command - 30,))
    shape = target_request[2] if target_request else 0
    candidates = tracking_candidates(detections, img, yolo.source, wanted,
                                     yolo_config.CLASS_COLORS,
                                     yolo_config.CLASS_SHAPES, shape)
    result = tracker.update(candidates, now)
    sx, sy = canvas.width()/320, canvas.height()/240
    if result[0]:
        _, color, cx, cy, width, height = result
        canvas.draw_rectangle((int((cx-width//2)*sx),int((cy-height//2)*sy),
                               int(width*sx),int(height*sy)),color=(255,200,0),thickness=3)
        canvas.draw_cross(int(cx*sx),int(cy*sy),color=(255,200,0),size=10)
    canvas.draw_cross(canvas.width()//2,canvas.height()//2,color=(255,255,255),size=8)
    if tracking_last_send is None or time.ticks_diff(now, tracking_last_send) >= TRACK_SEND_PERIOD_MS:
        if target_request:
            uart.write(target_tracking_packet(tracking_sequence, target_request[0], shape, result))
        else:
            uart.write(tracking_packet(tracking_sequence, result))
        tracking_sequence = (tracking_sequence+1) & 0xFF
        tracking_last_send = now


def report_target_unavailable(canvas):
    """A shape request cannot fall back to color blobs if YOLO is disabled."""
    global tracking_sequence, tracking_last_send
    now = time.ticks_ms()
    canvas.draw_string_advanced(4, 4, 24, '任务需要启用 YOLO 模型', color=(255, 255, 255))
    if tracking_last_send is None or time.ticks_diff(now, tracking_last_send) >= TRACK_SEND_PERIOD_MS:
        uart.write(target_tracking_packet(tracking_sequence, target_request[0],
                                          target_request[2], (0, 0, 0, 0, 0, 0)))
        tracking_sequence = (tracking_sequence+1) & 0xFF
        tracking_last_send = now


# Seven-color preview uses a separate full-resolution drawing canvas.
def draw_preview_boxes(img):
    detected = detect_materials(img)
    canvas = preview_image()
    sx = canvas.width() / img.width()
    sy = canvas.height() / img.height()
    for _, index, rect, cx, cy in detected:
        name, _, color, _ = COLOR_PROFILES[index]
        x, y, w, h = rect
        x, y, w, h = int(x * sx), int(y * sy), int(w * sx), int(h * sy)
        canvas.draw_rectangle((x, y, w, h), color=color, thickness=2)
        canvas.draw_cross(int(cx * sx), int(cy * sy), color=color, size=8)
        canvas.draw_string_advanced(x, max(0, y - 26), 24, name, color=color)
    canvas.draw_cross(canvas.width() // 2, canvas.height() // 2,
                      color=(255, 255, 255), size=8)
    if not detected:
        canvas.draw_string_advanced(4, 4, 24, '未检测到物料', color=(255, 255, 255))
    return canvas


def find_max_circle(circles):
    max_size = 0
    max_circle = None
    for circle in circles:
        if circle.r() > max_size:
            max_circle = circle
            max_size = circle.r()
    return max_circle


def ring(a):
    global cached_ring_x, cached_ring_y, cached_ring_valid

    # 使用QQVGA进行检测，不切换回QVGA（避免卡顿）
    set_frame_size(sensor.QQVGA)
    sum_x = 0
    sum_y = 0
    count = 0

    img = sensor.snapshot().lens_corr(1.0)
    circles = img.find_circles(threshold=HOUGH_THRESHOLD, x_margin=10, y_margin=10,
                               r_margin=10, r_min=MIN_RADIUS, r_max=MAX_RADIUS)

    if circles:
        max_circle = find_max_circle(circles)
        if max_circle:
            area = (max_circle.x() - max_circle.r(), max_circle.y() - max_circle.r(), 2 * max_circle.r(), 2 * max_circle.r())

            target_threshold = None
            if a == "r":
                target_threshold = red_circle_threshold
            elif a == "g":
                target_threshold = green_circle_threshold
            elif a == "b":
                target_threshold = blue_circle_threshold

            if target_threshold:
                blobs = img.find_blobs([target_threshold], roi=area)
                if blobs:
                    x, y, r = max_circle.x(), max_circle.y(), max_circle.r()
                    Z = np.array([x, y, 2*r, 2*r, 0, 0])
                    state = kf.update(Z)
                    pred_x = int(state[0])
                    pred_y = int(state[1])
                    pred_r = int(state[2]/2)

                    img.draw_rectangle(area, color=(0, 255, 0), thickness=2)
                    img.draw_circle(x, y, r, color=(0,255,0), thickness=2)
                    img.draw_cross(pred_x, pred_y, color=(255,0,0), size=10)
                    img.draw_circle(pred_x, pred_y, pred_r, color=(255,255,0))

                    sum_x = pred_x
                    sum_y = pred_y
                    count = 1

    show_image(img)
    if count > 0:
        avg_x = sum_x // count
        avg_y = sum_y // count

        # 缓存QQVGA坐标，命令7会将其转换为QVGA坐标
        cached_ring_x = avg_x
        cached_ring_y = avg_y
        cached_ring_valid = True

        if a == "r":
            send_data_adjustment_Colorround(avg_x, avg_y)
            print("Sent R:", avg_x, avg_y)
        elif a == "g":
            send_data_adjustment_Colorround(avg_x, avg_y)
            print("Sent G (cached):", avg_x, avg_y)
        elif a == "b":
            send_data_adjustment_Colorround(avg_x, avg_y)
            print("Sent B:", avg_x, avg_y)
    else:
        cached_ring_valid = False
        print("No matching circles found.")
skip_color_argument = False
try:
    if yolo_config.ENABLED:
        yolo = YoloDetector(yolo_config, source=sensor.VGA)
    sensor.init(yolo_enabled=yolo is not None)
    uart = create_uart()
    set_light(False)
    while(True):
        clock.tick()
        img = sensor.snapshot()  # 去掉畸变校正，恢复正常视野

        if uart.any():
            received_data = uart.read()
            for event in command_parser.feed(received_data or b'', time.ticks_ms()):
                if event[0] == 'target':
                    target_request = event[1:]
                    data = [35, 0]
                    tracker.reset()
                    tracking_last_send = None
                    cached_ring_valid = False
                    skip_color_argument = False
                    set_light(True)
                    continue
                command = event[1]
                if skip_color_argument:
                    skip_color_argument = False
                    continue
                if command == ord('a'):
                    set_light(True)
                elif command == ord('b'):
                    set_light(False)
                elif data[0] == 9 and command == 0x04:
                    skip_color_argument = True  # Existing command-9 acknowledgement.
                elif command in (1, 2, 3, 5, 6, 7, 8, 9, 21, 22, 23, 30, 31, 32, 33, 34):
                    target_request = None
                    new_command = 0 if command == 30 else command
                    if new_command != data[0]:
                        tracker.reset()
                        tracking_last_send = None
                        if data[0] in (31, 32, 33, 34) or new_command in (0, 31, 32, 33, 34):
                            cached_ring_valid = False
                    data = [new_command, 0]

        if yolo is not None and data[0] in (0, 31, 32, 33, 34, 35):
            set_frame_size(sensor.QVGA)
            # KPU planar channel is separate from clean YUV video and LAB RGB565.
            detections = yolo.detect(sensor.snapshot_yolo())
            canvas = preview_image()
            draw_detections(canvas, detections, yolo.source, yolo_config.CLASS_NAMES)
            if data[0] != 0:
                img = sensor.snapshot()
                run_yolo_tracking(img, detections, canvas, data[0])
            show_image(canvas)
            continue

        if data[0] == 35:
            canvas = preview_image()
            report_target_unavailable(canvas)
            show_image(canvas)
            continue

        if data[0] in (31, 32, 33, 34):
            set_frame_size(sensor.QVGA)
            if img.width() != 320 or img.height() != 240:
                img = sensor.snapshot()
            run_tracking(img, data[0])
            show_image(img)
            continue

        if data[0] != 0:
            if data[0] == 1 or data[0] == 2 or data[0] == 3:
                # 确保使用QVGA分辨率（避免之前命令5改成QQVGA）
                set_frame_size(sensor.QVGA)
                img = sensor.snapshot()  # 重新拍照，使用正确的分辨率

                # 强光环境优化：降低阈值，增加margin容差
                blobs = img.find_blobs([thresholds[data[0] - 1]], roi=Block_Roi,
                                       pixels_threshold=800, area_threshold=600,
                                       merge=True, margin=15)

                # 调试模式：打印检测到的色块LAB值
                if DEBUG_MODE and blobs:
                    color_name = ["红", "绿", "蓝"][data[0] - 1]
                    print("===CMD", data[0], color_name, "===")
                    for i, blob in enumerate(blobs):
                        stats = img.get_statistics(roi=(blob.x(), blob.y(), blob.w(), blob.h()))
                        print("BLOB%d: L=%d A=%d B=%d" % (i+1, stats.l_mean(), stats.a_mean(), stats.b_mean()))

                for blob in blobs:
                    img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                    img.draw_cross(blob.cx(), blob.cy())
                    print(blob.cx(), blob.cy())
                    if (blob.cx() - 133 < 120) and (blob.cy() - 120 < 120) and (blob.cx() - 133 > -120) and (blob.cy() - 120 > -120):
                        send_data_Colorround(data[0])
                        print(data[0])
            if data[0] == 5:
                ring("g")
            if data[0] == 6 or data[0] == 7 or data[0] == 8:
                # 确保使用QVGA分辨率进行色块检测
                set_frame_size(sensor.QVGA)
                img = sensor.snapshot()  # 重新拍照，使用正确的分辨率

                # 命令7首次可使用缓存坐标（仅使用一次），之后清除缓存进行实时检测
                if data[0] == 7 and cached_ring_valid:
                    # QQVGA(160x120) -> QVGA(320x240)，坐标需要x2
                    qvga_x = cached_ring_x * 2
                    qvga_y = cached_ring_y * 2
                    send(qvga_x, qvga_y)
                    print("命令7使用缓存坐标(转换后):", qvga_x, qvga_y)
                    # 使用一次后清除缓存，后续命令7将实时检测
                    cached_ring_valid = False
                else:
                    # 正常检测
                    blobs = img.find_blobs([thresholds[data[0] - 6]], roi=Block_Roi,
                                           pixels_threshold=800, area_threshold=600,
                                           merge=True, margin=15)

                    # 调试模式：打印检测到的色块LAB值
                    if DEBUG_MODE and blobs:
                        color_name = ["红", "绿", "蓝"][data[0] - 6]
                        print("===CMD", data[0], color_name, "===")
                        for i, blob in enumerate(blobs):
                            stats = img.get_statistics(roi=(blob.x(), blob.y(), blob.w(), blob.h()))
                            print("BLOB%d: L=%d A=%d B=%d" % (i+1, stats.l_mean(), stats.a_mean(), stats.b_mean()))

                    for blob in blobs:
                        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                        img.draw_cross(blob.cx(), blob.cy())
                        send(blob.cx(), blob.cy())
                        print(blob.cx(), blob.cy())
            if  data[0] == 21 or data[0] == 22 or data[0] == 23:
                set_frame_size(sensor.QVGA)
                if img.width() != 320:
                    img = sensor.snapshot()
                blobs = img.find_blobs([thresholds[data[0] - 21]], roi=Block_Roi,
                                       pixels_threshold=800, area_threshold=600,
                                       merge=True, margin=15)
                # 添加调试信息
                print("命令", data[0], "检测: 找到", len(blobs), "个色块")
                if blobs:
                    found_in_center = False
                    for blob in blobs:
                        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                        img.draw_cross(blob.cx(), blob.cy())
                        if (blob.cx() - 125 < 60) and (blob.cy() - 108 < 60) and (blob.cx() - 125 > -60) and (blob.cy() - 108 > -60):
                            send_data_Colorround_cmd21_23(data[0])  # 在中心，发送21/22/23
                            print("在中心!确认码:", data[0])
                            found_in_center = True
                            break
                    if not found_in_center:
                        send_data_Colorround_cmd21_23(55)  # 检测到但不在中心，发送55
                        print("不在中心，发送55")
                else:
                    send_data_Colorround_cmd21_23(66)  # 未检测到，发送66
                    print("未检测到命令", data[0], "，发送66")
            if data[0] == 9:
                set_frame_size(sensor.QVGA)
                if img.width() != 320:
                    img = sensor.snapshot()
                # 物料检测逻辑 - 校准后直接可抓取，无需检测目标色
                # 简化命令9逻辑：只做看到/没看到，具体流程由下位机处理
                # 简单视觉逻辑：只做多色块识别，send2格式发送结果
                blobs_1 = img.find_blobs([thresholds[0]], roi=Block_Roi,
                                         pixels_threshold=800, area_threshold=600,
                                         merge=True, margin=15)
                blobs_2 = img.find_blobs([thresholds[1]], roi=Block_Roi,
                                         pixels_threshold=800, area_threshold=600,
                                         merge=True, margin=15)
                blobs_3 = img.find_blobs([thresholds[2]], roi=Block_Roi,
                                         pixels_threshold=800, area_threshold=600,
                                         merge=True, margin=15)

                print("命令9检测: 红", len(blobs_1), "绿", len(blobs_2), "蓝", len(blobs_3))

                for group in (blobs_1, blobs_2, blobs_3):
                    for blob in group:
                        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                        img.draw_cross(blob.cx(), blob.cy())

                detected_color = 0
                if blobs_1:
                    detected_color = 1
                elif blobs_2:
                    detected_color = 2
                elif blobs_3:
                    detected_color = 3

                if detected_color == 0:
                    send2(88, 0, 0, 0)
                    print("未检测到，发送88,0")
                    time.sleep_ms(100)  # 延时100ms，避免发送太快
                else:
                    send2(77, detected_color, 0, 0)
                    print("检测到颜色", detected_color, "发送77,", detected_color)
                    time.sleep_ms(100)  # 延时100ms，避免发送太快
        if data[0] == 0:
            set_frame_size(sensor.QVGA)
            if img.width() != 320 or img.height() != 240:
                img = sensor.snapshot()
            img = draw_preview_boxes(img)
        if data[0] != 5:  # ring() displays its own annotated frame.
            show_image(img)
except KeyboardInterrupt:
    print("Vision stopped")
finally:
    if yolo is not None:
        yolo.close()
    # Notify the STM32 of target loss before releasing camera/UART resources.
    if uart is not None:
        try:
            if target_request:
                uart.write(target_tracking_packet(tracking_sequence, target_request[0],
                                                  target_request[2], (0, 0, 0, 0, 0, 0)))
            else:
                uart.write(tracking_packet(tracking_sequence, (0, 0, 0, 0, 0, 0)))
        finally:
            shutdown(uart)
    else:
        shutdown()
