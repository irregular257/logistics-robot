"""640x480 seven-color preview. No chassis UART is opened."""
import os, time
from k230_hal import sensor, preview_image, show_image, shutdown
from vision_colors import COLOR_PROFILES, detect_materials

try:
    sensor.init()
    fps = time.clock()
    last_log = time.ticks_ms()
    while True:
        fps.tick()
        img = sensor.snapshot()
        detected = detect_materials(img)
        canvas = preview_image()
        sx = canvas.width() / img.width()
        sy = canvas.height() / img.height()
        for _, index, rect, cx, cy in detected:
            name, _, color, _ = COLOR_PROFILES[index]
            x, y, w, h = rect
            x, y, w, h = int(x*sx), int(y*sy), int(w*sx), int(h*sy)
            canvas.draw_rectangle((x,y,w,h), color=color, thickness=2)
            canvas.draw_cross(int(cx*sx), int(cy*sy), color=color, size=8)
            canvas.draw_string_advanced(x, max(0,y-26), 24, name, color=color)
        canvas.draw_cross(canvas.width()//2, canvas.height()//2,
                          color=(255,255,255), size=8)
        if not detected:
            canvas.draw_string_advanced(4,4,24,'未检测到物料',color=(255,255,255))
        show_image(canvas)
        now = time.ticks_ms()
        if time.ticks_diff(now,last_log) >= 3000:
            print('SEVEN_COLORS:', [COLOR_PROFILES[item[1]][0] for item in detected],
                  'preview:', canvas.width(), canvas.height(), 'FPS:', fps.fps())
            last_log = now
finally:
    shutdown()
