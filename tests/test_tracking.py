import ast
import ctypes
import math
import sys
import types
import unittest
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vision_tracking import ColorTracker, crc8, tracking_packet
from vision_ranging import VisualRanger, ranging_packet
import vision_range_config as range_config


class RangingTests(unittest.TestCase):
    def ranger(self, **kwargs):
        return VisualRanger((0, 300, 300, 300), (0, 60, 60, 60), alpha=1, **kwargs)

    def observation(self, width=20, color=1, left=100, top=100, height=20):
        return (left + width // 2, top + height // 2, width, height,
                width * height, color, (left, top, width, height))

    def test_reference_and_inverse_size_depth(self):
        ranger = self.ranger()
        self.assertEqual(ranger.measure(1, self.observation(60), (320, 240)), (1, 300, 0))
        self.assertEqual(ranger.measure(1, self.observation(30), (320, 240)), (1, 600, 0))
        self.assertEqual(ranger.measure(1, self.observation(20), (320, 240)), (1, 900, 0))

    def test_no_calibration_and_nonfinite_calibration(self):
        for reference in (0, -300, float('nan'), float('inf')):
            ranger = VisualRanger((0, reference, 0, 0), (0, 60, 0, 0))
            self.assertEqual(ranger.measure(1, self.observation(), (320, 240)), (0, 0, 2))
        for width in (0, 2, float('nan'), float('inf')):
            ranger = VisualRanger((0, 300, 0, 0), (0, width, 0, 0))
            self.assertEqual(ranger.measure(1, self.observation(), (320, 240)), (0, 0, 2))

    def test_loss_unconfirmed_resolution_and_limits(self):
        ranger = self.ranger()
        self.assertEqual(ranger.measure(0, self.observation(), (320, 240)), (0, 0, 1))
        self.assertEqual(ranger.measure(1, None, (320, 240)), (0, 0, 1))
        self.assertEqual(ranger.measure(1, self.observation(), (160, 120)), (0, 0, 5))
        ranger = self.ranger(max_distance=500)
        self.assertEqual(ranger.measure(1, self.observation(), (320, 240)), (0, 0, 4))

    def test_clipped_borders_and_small_size(self):
        ranger = self.ranger()
        for observation in (self.observation(7), self.observation(left=0),
                            self.observation(top=0), self.observation(left=300),
                            self.observation(top=220)):
            self.assertEqual(ranger.measure(1, observation, (320, 240)), (0, 0, 3))
        # A centroid away from the edge cannot hide an actually clipped bbox.
        observation = (100, 100, 20, 20, 300, 1, (0, 100, 20, 20))
        self.assertEqual(ranger.measure(1, observation, (320, 240)), (0, 0, 3))

    def test_smoothing_resets_after_loss_and_color_change(self):
        ranger = VisualRanger((0, 300, 300, 0), (0, 60, 60, 0), alpha=0.5)
        self.assertEqual(ranger.measure(1, self.observation(20), (320, 240))[1], 900)
        self.assertEqual(ranger.measure(1, self.observation(40), (320, 240))[1], 675)
        ranger.measure(0, None, (320, 240))
        self.assertEqual(ranger.measure(1, self.observation(40), (320, 240))[1], 450)
        self.assertEqual(ranger.measure(1, self.observation(20, color=2), (320, 240))[1], 900)

    def test_tracker_preserves_current_bbox_not_smoothed_bbox(self):
        tracker = ColorTracker(confirm_frames=1)
        tracker.update([self.observation(20)], 0)
        current = self.observation(30, left=95)
        result = tracker.update([current], 50)
        self.assertEqual(tracker.observation, current)
        self.assertNotEqual(result[4], current[2])
        tracker.update([], 100)
        self.assertIsNone(tracker.observation)

    def test_range_wire_format(self):
        packet = ranging_packet(257, 2, (1, 600, 0))
        self.assertEqual(bytes(packet), bytes((0xF8, 1, 1, 2, 88, 2, 0,
                                               crc8(packet[:7]), 0xFE)))
        self.assertEqual(ranging_packet(1, 2, (0, 999, 2))[4:6], bytes(2))
        for result in ((1, 0, 0), (1, 65536, 0), (1, 100, 2)):
            with self.assertRaises(ValueError):
                ranging_packet(1, 1, result)


class TrackerTests(unittest.TestCase):
    def test_confirm_smooth_and_keep_identity(self):
        tracker = ColorTracker()
        target = (100, 120, 20, 20, 350, 1)
        self.assertEqual(tracker.update([target], 0)[0], 0)
        self.assertEqual(tracker.update([target], 50)[0], 0)
        result = tracker.update([(110, 120, 20, 20, 350, 1),
                                 (250, 120, 40, 40, 1500, 1)], 100)
        self.assertEqual(result[0], 1)
        self.assertTrue(100 < result[2] < 110)
        self.assertEqual(result[1], 1)

    def test_loss_never_publishes_prediction_and_reacquires(self):
        tracker = ColorTracker(confirm_frames=1)
        tracker.update([(100, 100, 20, 20, 350, 1)], 0)
        self.assertEqual(tracker.update([], 50), (0, 0, 0, 0, 0, 0))
        # A distant larger distractor cannot immediately take over the lock.
        self.assertEqual(tracker.update([(290, 100, 20, 20, 350, 1)], 100)[0], 0)
        self.assertEqual(tracker.update([(290, 100, 20, 20, 350, 1)], 500)[2], 290)

    def test_size_and_color_gate(self):
        tracker = ColorTracker(confirm_frames=1)
        tracker.update([(100, 100, 20, 20, 350, 1)], 0)
        self.assertEqual(tracker.update([(102, 100, 20, 20, 350, 2),
                                         (101, 100, 100, 100, 9000, 1)], 50)[0], 0)

    def test_packet_full_width_and_loss(self):
        self.assertEqual(crc8(b'123456789'), 0xF4)
        packet = tracking_packet(255, (1, 3, 319, 239, 320, 240))
        self.assertEqual(len(packet), 14)
        self.assertEqual(packet[4:6], bytes((63, 1)))
        self.assertEqual(packet[12], crc8(packet[:12]))
        self.assertEqual(tracking_packet(256, (0, 3, 319, 239, 320, 240))[1:12], bytes(11))

    def test_tick_wrap(self):
        diff = lambda a, b: ((a - b + (1 << 29)) % (1 << 30)) - (1 << 29)
        tracker = ColorTracker(ticks_diff=diff, confirm_frames=1)
        tracker.update([(100, 100, 20, 20, 350, 1)], (1 << 30) - 20)
        self.assertEqual(tracker.update([(105, 100, 20, 20, 350, 1)], 30)[0], 1)

    def test_entry_source_syntax_and_unsigned_formats(self):
        tree = ast.parse((ROOT / 'vision(3).py').read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == 'pack':
                self.assertNotIn('b', node.args[0].value)

    def test_k230_loop_sends_only_positions_and_lost_frames(self):
        import numpy

        class EndLoop(Exception):
            pass

        class Blob:
            def cx(self): return 290
            def cy(self): return 120
            def w(self): return 20
            def h(self): return 20
            def pixels(self): return 350
            def rect(self): return (280, 110, 20, 20)

        class Frame:
            def width(self): return 320
            def height(self): return 240
            def find_blobs(self, *args, **kwargs):
                return [Blob()] if state['frame'] <= 3 else []
            def draw_rectangle(self, *args, **kwargs): pass
            def draw_cross(self, *args, **kwargs): pass
            def draw_string_advanced(self, *args, **kwargs): state['labels'].append(args[3])

        state = {'frame': 0, 'read': False, 'packets': [], 'labels': [], 'cleanup': 0}
        sensor = types.ModuleType('sensor')
        sensor.RGB565, sensor.QVGA, sensor.QQVGA = 0, 1, 2
        for name in ('reset', 'set_pixformat', 'set_framesize', 'skip_frames', 'set_auto_gain',
                     'set_auto_whitebal', 'set_auto_exposure', 'set_hmirror', 'set_vflip', 'set_windowing'):
            setattr(sensor, name, lambda *args, **kwargs: None)

        def snapshot():
            state['frame'] += 1
            if state['frame'] > 4:
                raise EndLoop()
            return Frame()

        sensor.snapshot = snapshot
        sensor.init = lambda **kwargs: None

        class UART:
            def __init__(self, *args, **kwargs): pass
            def any(self): return not state['read']
            def read(self):
                state['read'] = True
                return bytes((ord('a'), 31))
            def write(self, data): state['packets'].append(bytes(data))

        def shutdown(uart=None): state['cleanup'] += 1

        fake_time = types.ModuleType('time')
        fake_time.clock = lambda: types.SimpleNamespace(tick=lambda: None)
        fake_time.ticks_ms = lambda: state['frame'] * 50
        fake_time.ticks_diff = lambda a, b: a - b
        fake_time.sleep_ms = lambda *args: None
        modules = {'ustruct': __import__('struct'), 'time': fake_time,
                   'k230_hal': types.SimpleNamespace(sensor=sensor, create_uart=UART,
                           set_light=lambda enabled: None, show_image=lambda img: None,
                           preview_image=lambda img: img,
                           draw_text=lambda img,x,y,text,*args,**kwargs: state['labels'].append(text),
                           shutdown=shutdown),
                   'ulab': types.SimpleNamespace(numpy=numpy),
                   'yolo_config': types.SimpleNamespace(ENABLED=False)}
        source = (ROOT / 'vision(3).py').read_text(encoding='utf-8')
        # Ranging modules are no longer dependencies of the deployed entry.
        modules['vision_ranging'] = None
        modules['vision_range_config'] = None
        with patch.dict(sys.modules, modules):
            with self.assertRaises(EndLoop):
                exec(compile(source, str(ROOT / 'vision(3).py'), 'exec'), {})
        self.assertEqual([packet[0] for packet in state['packets']], [0xF9] * 5)
        self.assertEqual(state['cleanup'], 1)
        self.assertEqual(state['packets'][-1][2], 0)
        packets = state['packets'][:4]
        self.assertEqual([packet[2] for packet in packets], [0, 0, 1, 0])
        self.assertEqual(packets[2][4] | packets[2][5] << 8, 290)
        self.assertEqual(packets[2][12], crc8(packets[2][:12]))
        self.assertEqual(state['labels'][2], 'Y=120px dY=0px')
        self.assertEqual(state['labels'][3], 'Target lost')


class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fw = ctypes.CDLL(str(ROOT / 'tests' / '.build' / 'vision_follow_host.dll'))
        for name in ('host_speed', 'host_angle', 'host_omega'):
            getattr(cls.fw, name).restype = ctypes.c_float
        cls.fw.host_time.argtypes = [ctypes.c_uint32]
        cls.fw.host_yaw.argtypes = [ctypes.c_float]

    def setUp(self):
        self.fw.host_reset()

    def enter(self, color=1):
        self.fw.host_mode(1, color)
        self.assertEqual(self.fw.host_step(), 1)

    def feed(self, data):
        for byte in data:
            self.fw.host_byte(byte)

    def packet(self, sequence=0, valid=1, color=1, x=100, y=120):
        self.feed(tracking_packet(sequence, (valid, color, x, y, 20, 20)))

    def test_modes_and_busy_route(self):
        self.assertEqual(self.fw.host_step(), 0)
        self.fw.host_idle(0)
        self.fw.host_mode(1, 1)
        self.assertEqual(self.fw.host_step(), 0)
        self.assertEqual(self.fw.host_state(), 4)
        self.assertEqual(self.fw.host_enabled(), 0)

    def test_observation_only_never_drives_chassis_and_rejects_stale_samples(self):
        self.assertEqual(self.fw.host_observe_start(4),1)
        self.assertEqual(self.fw.host_observe_read(),0)
        self.packet(1,x=165,y=133)
        self.assertEqual(self.fw.host_observe_read(),1)
        self.fw.host_key(1)
        self.fw.host_time(60)
        self.fw.host_step()
        self.assertEqual(self.fw.host_calls(),0)
        self.fw.host_time(251)
        self.assertEqual(self.fw.host_observe_read(),0)
        self.assertEqual(self.fw.host_observe_stop(),1)
        self.assertEqual(self.fw.host_observe_read(),0)
        self.assertEqual(self.fw.host_command(2),30)

    def test_crc_coordinates_direction_limit_and_watchdog(self):
        self.enter()
        corrupt = tracking_packet(0, (1, 1, 100, 120, 20, 20))
        corrupt[12] ^= 1
        self.feed(corrupt)
        self.assertEqual(self.fw.host_seen(), 0)
        self.packet(1, x=319, y=239)
        self.assertEqual(self.fw.host_x(), 319)
        self.assertEqual(self.fw.host_y(), 239)
        self.fw.host_key(1)
        self.fw.host_time(60)
        self.fw.host_step()
        speed = self.fw.host_speed()
        self.assertAlmostEqual(speed, 0.12, places=6)
        angle = math.radians(self.fw.host_angle())
        self.assertLess(speed * math.cos(angle), 0)
        self.assertGreater(speed * math.sin(angle), 0)
        self.fw.host_time(251)
        self.fw.host_step()
        self.assertEqual(self.fw.host_speed(), 0)
        self.assertEqual(self.fw.host_state(), 3)

    def test_deadband_and_immediate_lost_stop(self):
        self.enter()
        self.fw.host_key(1)
        self.packet(x=164, y=116)
        self.fw.host_time(60)
        self.fw.host_step()
        self.assertEqual(self.fw.host_speed(), 0)
        self.fw.host_time(120)
        self.packet(1, x=100)
        self.fw.host_step()
        self.assertGreater(self.fw.host_speed(), 0)
        self.packet(2, valid=0)
        self.fw.host_step()
        self.assertEqual(self.fw.host_speed(), 0)

    def test_key_release_stop_and_exit_guard(self):
        self.enter()
        self.fw.host_key(1)
        self.packet(x=100)
        self.fw.host_time(60)
        self.fw.host_step()
        self.fw.host_key(0)
        self.fw.host_step()
        self.assertEqual(self.fw.host_speed(), 0)
        self.fw.host_key(1)
        self.fw.host_mode(0, 1)
        self.assertEqual(self.fw.host_step(), 1)
        self.assertEqual(self.fw.host_command(2), 30)
        self.assertEqual(self.fw.host_step(), 1)
        self.fw.host_key(0)
        self.assertEqual(self.fw.host_step(), 1)
        self.assertEqual(self.fw.host_step(), 0)

    def test_legacy_frame_boundaries_and_confirmation(self):
        self.feed(bytes((0xFF, 165, 133, 0xFE, 0x01, 80, 66, 0xFE)))
        self.assertEqual(self.fw.host_column(0), 165)
        self.assertEqual(self.fw.host_column(1), 133)
        self.assertEqual(self.fw.host_ring(0), 80)
        self.feed(bytes((0x03, 1)))
        self.assertEqual(self.fw.host_confirmed(), 0)
        self.feed(bytes((0xFE,)))
        self.assertEqual(self.fw.host_confirmed(), 1)
        self.feed(bytes((0xFA, 77, 3, 0, 0, 0xFE)))
        self.assertEqual(self.fw.host_column(0), 77)
        self.assertEqual(self.fw.host_column(1), 3)

    def test_partial_packet_timeout_recovery(self):
        self.enter()
        self.feed(bytes((0xF9, 3, 1)))
        self.fw.host_time(31)
        self.packet(4)
        self.assertEqual(self.fw.host_seen(), 1)
        self.assertEqual(self.fw.host_x(), 100)

    def test_duplicate_frames_do_not_extend_watchdog(self):
        self.enter()
        self.packet(5)
        self.fw.host_key(1)
        self.fw.host_time(60)
        self.fw.host_step()
        self.fw.host_time(240)
        self.packet(5)
        self.fw.host_time(251)
        self.fw.host_step()
        self.assertEqual(self.fw.host_speed(), 0)
        self.assertEqual(self.fw.host_state(), 3)

    def test_color_change_and_wrong_color(self):
        self.enter(2)
        self.packet(color=1)
        self.assertEqual(self.fw.host_seen(), 0)
        self.packet(1, color=2)
        self.assertEqual(self.fw.host_seen(), 1)
        self.fw.host_mode(1, 3)
        self.fw.host_step()
        self.assertEqual(self.fw.host_seen(), 0)
        self.assertEqual(self.fw.host_command(3), 33)

    def test_distance_received_without_starting_motors(self):
        self.enter()
        self.packet(1)
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 1)
        self.assertEqual(self.fw.host_distance(), 900)
        self.assertEqual(self.fw.host_distance_reason(), 0)
        self.assertEqual(self.fw.host_speed(), 0)
        self.fw.host_time(251)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.assertEqual(self.fw.host_distance(), 0)
        self.assertEqual(self.fw.host_distance_reason(), 7)

    def test_distance_crc_sequence_color_and_duplicate_rejection(self):
        self.enter()
        # A range alone cannot create a target.
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.packet(1)
        bad = ranging_packet(1, 1, (1, 900, 0))
        bad[7] ^= 1
        self.feed(bad)
        self.feed(ranging_packet(0, 1, (1, 800, 0)))
        self.feed(ranging_packet(1, 2, (1, 700, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.feed(ranging_packet(1, 1, (1, 500, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance(), 900)
        self.fw.host_time(240)
        self.packet(1)
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.fw.host_time(251)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        # Delayed range cannot restore a stale sample.
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)

    def test_distance_requires_each_new_observation_and_clears_on_loss(self):
        self.enter()
        self.packet(255)
        self.feed(ranging_packet(255, 1, (1, 900, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 1)
        # Sequence wrap and missing range must invalidate the old distance.
        self.packet(0)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.assertEqual(self.fw.host_distance_reason(), 6)
        self.feed(ranging_packet(0, 1, (0, 0, 2)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_reason(), 2)
        self.packet(1)
        self.feed(ranging_packet(1, 1, (1, 800, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance(), 800)
        self.packet(2, valid=0)
        self.feed(ranging_packet(2, 0, (1, 900, 0)))  # Cannot range a lost target.
        self.feed(ranging_packet(2, 0, (0, 0, 1)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.assertEqual(self.fw.host_distance(), 0)
        self.assertEqual(self.fw.host_distance_reason(), 1)

    def test_distance_invalidates_on_mode_switch_and_exit(self):
        self.enter()
        self.packet(1)
        self.feed(ranging_packet(1, 1, (1, 900, 0)))
        self.fw.host_step()
        self.fw.host_mode(1, 2)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.packet(2, color=2)
        self.feed(ranging_packet(2, 2, (1, 600, 0)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance(), 600)
        self.fw.host_mode(0, 2)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)
        self.assertEqual(self.fw.host_distance(), 0)

    def test_interleaved_ranges_preserve_legacy_protocol(self):
        self.enter()
        self.packet(1)
        self.feed(ranging_packet(1, 1, (1, 254, 0)))  # FE is allowed inside payload.
        self.feed(bytes((0xFA, 77, 3, 0, 0, 0xFE)))
        self.fw.host_step()
        self.assertEqual(self.fw.host_column(0), 77)
        self.assertEqual(self.fw.host_column(1), 3)
        self.assertEqual(self.fw.host_distance(), 254)

    def test_range_semantic_validation_and_timeout_wrap(self):
        self.enter()
        self.packet(1)
        for valid, mm, reason in ((2, 600, 0), (1, 600, 2), (1, 0, 0),
                                  (0, 600, 2), (0, 0, 0), (0, 0, 6)):
            frame = bytearray((0xF8, 1, valid, 1, mm & 255, mm >> 8, reason))
            frame.extend((crc8(frame), 0xFE))
            self.feed(frame)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_reason(), 6)
        self.fw.host_time(0xFFFFFFF0)
        self.packet(2)
        self.feed(ranging_packet(2, 1, (1, 600, 0)))
        self.fw.host_time(0x20)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 1)
        self.fw.host_time(0x100)
        self.fw.host_step()
        self.assertEqual(self.fw.host_distance_valid(), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
