"""Exercise the actual STM32 vision parser through the compiled host DLL."""
import ctypes
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision_tracking import crc8, tracking_packet

DLL = Path(__file__).resolve().parent / '.build' / 'vision_follow_host.dll'


class MaterialWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.host = ctypes.CDLL(str(DLL))
        cls.host.host_time.argtypes = [ctypes.c_uint32]

    def setUp(self):
        self.host.host_reset()
        self.host.host_time(100)

    def send(self, wire):
        for byte in wire:
            self.host.host_byte(byte)

    def response(self, token=1, sequence=1, color=4, shape=1, valid=1, x=165):
        body = bytes((0xF6, sequence, valid, color if valid else 0,
                      shape if valid else 0, token))
        body += struct.pack('<4H', x, 133, 40, 50) if valid else bytes(8)
        return body + bytes((crc8(body), 0xFE))

    def test_request_encoding_and_black_is_not_legacy_any(self):
        self.assertEqual(self.host.host_target_start(4, 1), 1)
        actual = bytes(self.host.host_command(i) for i in range(6))
        expected = bytes((0xF7, 1, 4, 1))
        self.assertEqual(actual, expected + bytes((crc8(expected), 0xFE)))
        self.send(self.response())
        self.assertEqual(self.host.host_observe_read(), 1)
        self.assertEqual(self.host.host_target_color(), 4)
        self.assertEqual(self.host.host_target_shape(), 1)
        self.assertEqual(self.host.host_calls(), 0)

    def test_actual_color_and_shape_required(self):
        self.host.host_target_start(4, 1)
        for wire in (self.response(color=1), self.response(shape=3),
                     tracking_packet(3, (1, 1, 165, 133, 40, 50))):
            self.send(wire)
            self.assertEqual(self.host.host_observe_read(), 0)
        self.send(self.response(sequence=4))
        self.assertEqual(self.host.host_observe_read(), 1)

    def test_invalid_request_has_no_uart_or_mode_side_effect(self):
        for color, shape in ((5, 3), (6, 3), (1, 2), (7, 3), (1, 1), (3, 4), (1, 0)):
            self.assertEqual(self.host.host_target_start(color, shape), 0)
        self.assertEqual(self.host.host_command(0), -1)
        self.assertEqual(self.host.host_observe_read(), 0)

    def test_new_request_rejects_late_old_target_even_same_attributes(self):
        self.host.host_target_start(4, 1)
        self.send(self.response())
        self.host.host_observe_stop()
        self.host.host_target_start(4, 1)
        self.send(self.response(token=1, sequence=2))
        self.assertEqual(self.host.host_observe_read(), 0)
        self.send(self.response(token=2, sequence=3))
        self.assertEqual(self.host.host_observe_read(), 1)

    def test_crc_truncation_geometry_and_loss(self):
        self.host.host_target_start(4, 1)
        bad = bytearray(self.response()); bad[-2] ^= 1
        self.send(bad)
        self.assertEqual(self.host.host_observe_read(), 0)
        self.send(self.response(sequence=2, x=320))
        self.assertEqual(self.host.host_observe_read(), 0)
        wire = self.response(sequence=3)
        self.send(wire[:7]); self.assertEqual(self.host.host_observe_read(), 0)
        self.send(wire[7:]); self.assertEqual(self.host.host_observe_read(), 1)
        self.send(self.response(sequence=4, valid=0))
        self.assertEqual(self.host.host_observe_read(), 0)

    def test_replay_cannot_keep_target_fresh(self):
        self.host.host_target_start(7, 4)
        wire = self.response(color=7, shape=4)
        self.send(wire)
        self.assertEqual(self.host.host_observe_read(), 1)
        self.host.host_time(351)
        self.send(wire)
        self.assertEqual(self.host.host_observe_read(), 0)
        self.send(self.response(sequence=2, color=7, shape=4))
        self.assertEqual(self.host.host_observe_read(), 1)


if __name__ == '__main__':
    unittest.main()
