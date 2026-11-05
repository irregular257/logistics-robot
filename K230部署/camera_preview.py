"""Standalone IDE camera preview; does not communicate with the chassis."""
import os, time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

camera = None
display_ready = media_ready = False
try:
    os.exitpoint(os.EXITPOINT_ENABLE)
    camera = Sensor(width=640, height=480)
    camera.reset()
    camera.set_framesize(width=640, height=480)
    camera.set_pixformat(Sensor.RGB565)
    Display.init(Display.VIRT, width=640, height=480, fps=30, to_ide=True)
    display_ready = True
    MediaManager.init()
    media_ready = True
    camera.run()
    print('CAMERA_PREVIEW_RUNNING: IDE output enabled')
    while True:
        os.exitpoint()
        img = camera.snapshot()
        Display.show_image(img)
        time.sleep_ms(30)
except KeyboardInterrupt:
    print('Camera preview stopped')
finally:
    if camera is not None:
        camera.stop()
    if display_ready:
        Display.deinit()
    if media_ready:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()
