"""Run from CanMV IDE for YOLO preview only; does not open motor UART."""
import os, time
import yolo_config
from k230_hal import sensor, preview_image, show_image, shutdown
from vision_yolo import YoloDetector, draw_detections

yolo = None
try:
    yolo = YoloDetector(yolo_config, source=sensor.VGA)
    sensor.init(yolo_enabled=True)
    clock = time.clock()
    last_log = time.ticks_ms()
    while True:
        os.exitpoint()
        clock.tick()
        detections = yolo.detect(sensor.snapshot_yolo())
        canvas = preview_image()
        draw_detections(canvas,detections,yolo.source,yolo_config.CLASS_NAMES)
        show_image(canvas)
        now = time.ticks_ms()
        if time.ticks_diff(now,last_log) >= 3000:
            print('YOLO_PREVIEW_OK FPS:',clock.fps(),'detections:',detections)
            last_log = now
finally:
    if yolo is not None:
        yolo.close()
    shutdown()
