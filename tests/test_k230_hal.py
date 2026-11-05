import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import k230_config as config


class K230HalTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        calls = self.calls

        class FPIOA:
            UART3_TXD, UART3_RXD, PWM0 = 188, 187, 100
            def set_function(self, pin, function):
                calls.append(('map', pin, function))
                if (pin, function) not in ((32, 188), (33, 187), (42, 100)):
                    raise ValueError('Unsupported pin/function')

        class UART:
            EIGHTBITS, PARITY_NONE, STOPBITS_ONE = 8, 0, 1
            def __init__(self, uart_id, **kwargs): calls.append(('uart', uart_id, kwargs))
            def deinit(self): calls.append(('uart_close',))

        class Sensor:
            RGB565 = 1
            def __init__(self, **kwargs): calls.append(('camera', kwargs))
            def reset(self): calls.append(('reset',))
            def set_hmirror(self, value): calls.append(('mirror', value))
            def set_vflip(self, value): calls.append(('flip', value))
            def set_framesize(self, **kwargs): calls.append(('size', kwargs))
            def set_pixformat(self, fmt, **kwargs): calls.append(('format', fmt, kwargs))
            def run(self): calls.append(('run',))
            def snapshot(self, chn):
                calls.append(('snapshot', chn))
                return types.SimpleNamespace(width=lambda: 320 if chn == 0 else 160,
                                              height=lambda: 240 if chn == 0 else 120)
            def get_type(self): return 'GC2145'
            def stop(self): calls.append(('stop',))

        class Display:
            VIRT = 0
            @staticmethod
            def init(*args, **kwargs): calls.append(('display', kwargs))
            @staticmethod
            def deinit(): calls.append(('display_close',))
            @staticmethod
            def show_image(img, **kwargs): calls.append(('show', kwargs))

        class Manager:
            @staticmethod
            def init(): calls.append(('media',))
            @staticmethod
            def deinit(): calls.append(('media_close',))

        self.fake_os = types.SimpleNamespace(EXITPOINT_ENABLE=0, EXITPOINT_ENABLE_SLEEP=1,
                                             exitpoint=lambda *args: None)
        self.patches = patch.dict(sys.modules, {
            'machine': types.SimpleNamespace(FPIOA=FPIOA, UART=UART, PWM=lambda *a, **k: None),
            'media': types.ModuleType('media'),
            'media.sensor': types.SimpleNamespace(Sensor=Sensor),
            'media.display': types.SimpleNamespace(Display=Display),
            'media.media': types.SimpleNamespace(MediaManager=Manager),
        })
        self.patches.start()
        sys.modules.pop('k230_hal', None)
        self.hal = importlib.import_module('k230_hal')
        self.hal.os = self.fake_os
        self.hal.time = types.SimpleNamespace(sleep_ms=lambda ms: None)

    def tearDown(self):
        sys.modules.pop('k230_hal', None)
        self.patches.stop()

    def test_verified_uart3_maps_io32_io33_and_8n1(self):
        self.hal.create_uart()
        self.assertEqual(self.calls[:2], [('map', 32, 188), ('map', 33, 187)])
        self.assertEqual(self.calls[2], ('uart', 3, dict(baudrate=115200, bits=8, parity=0, stop=1)))

    def test_camera_channels_configured_before_run_and_switched_without_restart(self):
        camera = self.hal.sensor
        camera.init()
        self.assertIn(('display', dict(width=320, height=240,
                                        fps=config.PREVIEW_FPS, to_ide=True)), self.calls)
        sizes = [c for c in self.calls if c[0] == 'size']
        self.assertEqual(sizes, [('size', dict(width=320, height=240, chn=0)),
                                 ('size', dict(width=160, height=120, chn=1))])
        self.assertLess(self.calls.index(sizes[-1]), self.calls.index(('run',)))
        camera.set_framesize(camera.QQVGA)
        frame = camera.snapshot()
        self.assertEqual(frame.width(), 160)
        self.hal.show_image(frame)
        self.assertIn(('show', dict(x=80, y=60)), self.calls)
        camera.set_framesize(camera.QVGA)
        self.assertEqual(camera.snapshot().width(), 320)
        self.assertEqual(self.calls.count(('run',)), 1)

    def test_cleanup_after_stop_failure_still_releases_display_and_media(self):
        self.hal.sensor.init()
        self.hal.sensor.device.stop = lambda: (_ for _ in ()).throw(RuntimeError('camera error'))
        with self.assertRaises(RuntimeError):
            self.hal.shutdown()
        self.assertIn(('display_close',), self.calls)
        self.assertIn(('media_close',), self.calls)
        self.assertIsNone(self.hal.sensor.device)

    def test_unwired_light_does_not_claim_any_pin(self):
        self.hal.set_light(True)
        self.assertEqual(self.calls, [])
        with patch.object(config, 'LIGHT_PIN', 32):
            with self.assertRaises(ValueError):
                self.hal.set_light(True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
