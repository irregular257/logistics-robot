"""Yahboom K230 1G, automatic sensor detection, CanMV v1.4.3.

The IDE reports USB debug version 0.4.0; os.uname reports CanMV v1.4.3.
The connected camera was detected as GC2093 during the hardware test.
On this board IO42/43 are UART1 RTS/CTS, not TX/RX.
Wire IO32(TX) -> STM32 PC7(RX), IO33(RX) -> PC6(TX), GND -> GND.
"""
UART_ID = 3
UART_TX_PIN = 32
UART_RX_PIN = 33
UART_BAUDRATE = 115200
CAMERA_CAPTURE_WIDTH = 640
CAMERA_CAPTURE_HEIGHT = 480
CAMERA_HMIRROR = False
CAMERA_VFLIP = False
PREVIEW_FPS = 30

# Original OpenMV P6 has no established K230 wiring. Disabled until connected.
# IO42 supports PWM0 on the tested board; use a suitable LED driver circuit.
LIGHT_PIN = None
LIGHT_PWM_CHANNEL = 0
LIGHT_PWM_FREQUENCY = 500
LIGHT_DUTY_PERCENT = 45
