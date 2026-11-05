"""K230 camera, UART and optional light adapter for the original vision logic."""
import os, time, image
from machine import UART, FPIOA, PWM
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
import k230_config as config

_fpioa = FPIOA()
_light = None


def create_uart():
    tx = getattr(FPIOA, 'UART%d_TXD' % config.UART_ID)
    rx = getattr(FPIOA, 'UART%d_RXD' % config.UART_ID)
    _fpioa.set_function(config.UART_TX_PIN, tx)
    _fpioa.set_function(config.UART_RX_PIN, rx)
    uart = UART(config.UART_ID, baudrate=config.UART_BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)
    print('Vision UART%d: IO%d TX -> PC7, IO%d RX -> PC6, %d 8N1' %
          (config.UART_ID, config.UART_TX_PIN, config.UART_RX_PIN, config.UART_BAUDRATE))
    return uart


def set_light(enabled):
    global _light
    if config.LIGHT_PIN is None:
        return
    if config.LIGHT_PIN in (config.UART_TX_PIN, config.UART_RX_PIN):
        raise ValueError('Light pin conflicts with UART')
    if _light is None:
        _fpioa.set_function(config.LIGHT_PIN, getattr(FPIOA, 'PWM%d' % config.LIGHT_PWM_CHANNEL))
        _light = PWM(config.LIGHT_PWM_CHANNEL, config.LIGHT_PWM_FREQUENCY, 0, enable=True)
    _light.duty(config.LIGHT_DUTY_PERCENT if enabled else 0)


class Camera:
    VGA = (640, 480)
    QVGA = (320, 240)
    QQVGA = (160, 120)

    def __init__(self):
        self.yolo_enabled = False
        self.frame_size = self.QVGA
        self.device = None
        self.channel = 1
        self.display_started = False
        self.media_started = False
        self.canvas = None
        self.text_canvas = None
        self.preview_mode = False
        self.labels = []

    def init(self, yolo_enabled=False):
        self.yolo_enabled = yolo_enabled
        self.frame_size = self.QVGA
        os.exitpoint(os.EXITPOINT_ENABLE)
        self.device = Sensor(width=config.CAMERA_CAPTURE_WIDTH,
                             height=config.CAMERA_CAPTURE_HEIGHT)
        self.device.reset()
        self.device.set_hmirror(config.CAMERA_HMIRROR)
        self.device.set_vflip(config.CAMERA_VFLIP)
        # Preconfigure both sizes BEFORE run(); switch channels, not live ISP size.
        for channel, size in enumerate((self.QVGA, self.VGA if yolo_enabled else self.QQVGA), 1):
            self.device.set_framesize(width=size[0], height=size[1], chn=channel)
            self.device.set_pixformat(Sensor.RGBP888 if yolo_enabled and channel == 2 else Sensor.RGB565, chn=channel)
        self.device.set_framesize(width=config.PREVIEW_WIDTH, height=config.PREVIEW_HEIGHT, chn=0)
        self.device.set_pixformat(Sensor.YUV420SP, chn=0)
        Display.bind_layer(**self.device.bind_info(), layer=Display.LAYER_VIDEO1)
        Display.init(Display.VIRT, width=config.PREVIEW_WIDTH, height=config.PREVIEW_HEIGHT,
                     fps=config.PREVIEW_FPS, to_ide=True, osd_num=2)
        self.display_started = True
        MediaManager.init()
        self.media_started = True
        self.canvas = image.Image(config.PREVIEW_WIDTH, config.PREVIEW_HEIGHT, image.ARGB8888)
        self.text_canvas = image.Image(config.PREVIEW_WIDTH, config.PREVIEW_HEIGHT, image.ARGB8888)
        self.device.run()
        self.channel = 1
        print('Vision camera:', self.device.get_type(), '640x480 YUV preview + ARGB overlay; QVGA/QQVGA detection')

    def set_framesize(self, size):
        if size not in (self.QVGA, self.QQVGA):
            raise ValueError('Only calibrated QVGA and legacy QQVGA are supported')
        self.frame_size = size
        self.channel = 1 if size == self.QVGA or self.yolo_enabled else 2

    def snapshot(self):
        os.exitpoint()
        img = self.device.snapshot(chn=self.channel)
        if self.yolo_enabled and self.frame_size == self.QQVGA:
            return img.copy(x_scale=0.5, y_scale=0.5)
        return img

    def snapshot_yolo(self):
        os.exitpoint()
        if not self.yolo_enabled:
            raise ValueError('Initialize camera with yolo_enabled=True')
        return self.device.snapshot(chn=2)

    def preview_image(self):
        os.exitpoint()
        self.canvas.clear()
        self.labels = []
        return self.canvas

    def close(self):
        # Attempt all cleanup stages even after partial startup failure.
        try:
            if self.device is not None:
                self.device.stop()
        finally:
            self.device = None
            self.canvas = None
            self.text_canvas = None
            self.preview_mode = False
            self.labels = []
            try:
                if self.display_started:
                    Display.deinit()
            finally:
                self.display_started = False
                if self.media_started:
                    try:
                        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                        time.sleep_ms(100)
                        MediaManager.deinit()
                    finally:
                        self.media_started = False


sensor = Camera()


def preview_image():
    return sensor.preview_image()


def draw_text(x, y, size, text, color=(255, 255, 255)):
    # Never draw advanced text into a camera RGB565 buffer.
    sensor.labels.append((x, y, size, text, color))


def show_image(img):
    text_canvas = sensor.text_canvas
    text_canvas.clear()
    if img is sensor.canvas:
        # Hardware video is the clean background; this canvas contains only annotations.
        if not sensor.preview_mode:
            Display.show_image(text_canvas, layer=Display.LAYER_OSD1)
        Display.show_image(img, layer=Display.LAYER_OSD0)
        sensor.preview_mode = True
    else:
        sensor.preview_mode = False
        sx = config.PREVIEW_WIDTH / img.width()
        sy = config.PREVIEW_HEIGHT / img.height()
        scaled = img.copy(x_scale=sx, y_scale=sy)
        Display.show_image(scaled, layer=Display.LAYER_OSD0)
        for x, y, size, text, color in sensor.labels:
            text_canvas.draw_string_advanced(int(x*sx), int(y*sy), int(size*min(sx,sy)),
                                             text, color=color)
        Display.show_image(text_canvas, layer=Display.LAYER_OSD1)
    sensor.labels = []


def shutdown(uart=None):
    global _light
    try:
        sensor.close()
    finally:
        if _light is not None:
            _light.duty(0)
            _light.deinit()
            _light = None
        if uart is not None:
            uart.deinit()
