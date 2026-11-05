"""Host regression tests for color+shape selection and the K230 task UART."""
import ast
import ctypes
import sys
import types
import unittest
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / 'K230部署'
sys.path.insert(0, str(DEPLOY))
import yolo_config
from vision_tracking import (ColorTracker, TargetCommandParser, crc8,
                             target_supported, target_tracking_packet, tracking_packet)
from vision_yolo import tracking_candidates


def request(token=1, color=1, shape=3):
    data = bytearray((0xF7, token, color, shape))
    data.extend((crc8(data), 0xFE))
    return data


class TargetProtocolTests(unittest.TestCase):
    def test_every_uart_split_preserves_one_request(self):
        wire = request(247, 7, 4)  # Token can itself equal the frame header.
        for split in range(1, 6):
            parser = TargetCommandParser()
            self.assertEqual(parser.feed(wire[:split], 0), [])
            self.assertEqual(parser.feed(wire[split:], 50), [('target', 247, 7, 4)])

    def test_concatenated_requests_and_legacy_stop(self):
        parser = TargetCommandParser()
        events = parser.feed(b'a' + request(3, 4, 1) + request(4, 2, 3) + bytes((30, 31)), 0)
        self.assertEqual(events, [('legacy', 97), ('target', 3, 4, 1),
                                  ('target', 4, 2, 3), ('legacy', 30), ('legacy', 31)])

    def test_bad_crc_never_replays_payload_as_legacy_commands(self):
        bad = request(30, 1, 3)
        bad[4] ^= 1
        parser = TargetCommandParser()
        self.assertEqual(parser.feed(bad + bytes((3, 7, 9)), 0), [])
        self.assertEqual(parser.feed(request(2, 2, 4), 1), [('target', 2, 2, 4)])

    def test_recover_after_truncated_frame_and_following_header(self):
        parser = TargetCommandParser()
        self.assertEqual(parser.feed(request()[:3], 0), [])
        self.assertEqual(parser.feed(request(9, 4, 1), 50), [('target', 9, 4, 1)])

    def test_truncated_expiry_does_not_execute_late_color_bytes(self):
        parser = TargetCommandParser(gap_ms=100)
        self.assertEqual(parser.feed(request()[:2], 0), [])
        self.assertEqual(parser.feed(request()[2:], 101), [])
        self.assertEqual(parser.feed(bytes((30, 32)), 102), [('legacy', 30), ('legacy', 32)])

    def test_exact_supported_combinations(self):
        expected = {(4, 1)} | {(c, 3) for c in (1, 2, 3, 4)} | {(c, 4) for c in (1, 2, 7)}
        expected |= {(c, s) for c in (1, 2, 3, 4, 7) for s in (5, 6)}
        for color in range(9):
            for shape in range(8):
                self.assertEqual(target_supported(color, shape), (color, shape) in expected)
                parser = TargetCommandParser()
                events = parser.feed(request(1, color, shape), 0)
                self.assertEqual(bool(events), (color, shape) in expected)

    def test_response_has_shape_request_token_and_crc(self):
        frame = target_tracking_packet(17, 93, 4, (1, 7, 165, 133, 24, 32))
        self.assertEqual(len(frame), 16)
        self.assertEqual(frame[:6], bytes((0xF6, 17, 1, 7, 4, 93)))
        self.assertEqual(frame[6:14], bytes((165, 0, 133, 0, 24, 0, 32, 0)))
        self.assertEqual(frame[14], crc8(frame[:14]))
        self.assertEqual(frame[15], 0xFE)

    def test_invalid_or_unsupported_observations_clear_target_keep_token(self):
        for result, shape in (((0, 7, 100, 80, 20, 30), 4),
                              ((1, 7, 100, 80, 20, 30), 3),
                              ((1, 6, 100, 80, 20, 30), 3)):
            frame = target_tracking_packet(18, 91, shape, result)
            self.assertEqual(frame[2:5], bytes((0, 0, 0)))
            self.assertEqual(frame[5], 91)
            self.assertEqual(frame[6:14], bytes(8))

    def test_legacy_packet_unchanged(self):
        frame = tracking_packet(8, (1, 2, 165, 133, 24, 32))
        self.assertEqual(len(frame), 14)
        self.assertEqual(frame[:4], bytes((0xF9, 8, 1, 2)))
        self.assertEqual(frame[-2], crc8(frame[:-2]))


class TargetSelectionTests(unittest.TestCase):
    def frame(self, lab=(45, 55, 30)):
        return types.SimpleNamespace(width=lambda: 320, height=lambda: 240,
            get_statistics=lambda **kw: types.SimpleNamespace(l_mean=lambda: lab[0],
                a_mean=lambda: lab[1], b_mean=lambda: lab[2]))

    def candidates(self, detections, color, shape, lab=(45, 55, 30)):
        return tracking_candidates(detections, self.frame(lab), (640, 480), (color,),
                                   yolo_config.CLASS_COLORS, yolo_config.CLASS_SHAPES, shape)

    def detection(self, cls, size=80):
        return (cls, .9, (200, 160, size, size))

    def test_color_and_shape_both_required_even_with_larger_wrong_target(self):
        detections = [self.detection(0, 160), self.detection(19, 120), self.detection(18, 80)]
        result = self.candidates(detections, 4, 3)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], (120, 100, 40, 40, 1600, 4, (100, 80, 40, 40)))
        tracker = ColorTracker(confirm_frames=3)
        self.assertFalse(tracker.update(result, 0)[0])
        self.assertFalse(tracker.update(result, 50)[0])
        self.assertEqual(tracker.update(result, 100)[:2], (1, 4))

    def test_black_hexagonal_prism_and_yellow_pentagonal_prism(self):
        self.assertEqual(self.candidates([self.detection(0)], 4, 1)[0][5], 4)
        self.assertEqual(self.candidates([self.detection(3)], 7, 4)[0][5], 7)
        self.assertEqual(self.candidates([self.detection(3)], 1, 4), [])

    def test_excluded_classes_never_enter_task_candidates(self):
        excluded = (6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17)
        for cls in excluded:
            for shape in (1, 3, 4, 5, 6):
                for color in (1, 2, 3, 4, 7):
                    self.assertEqual(self.candidates([self.detection(cls)], color, shape), [])

    def test_cuboid_and_original_naiwa_label_use_lab_for_color(self):
        for cls, shape in ((10, 5), (22, 6)):
            self.assertEqual(self.candidates([self.detection(cls)], 1, shape)[0][5], 1)
            self.assertEqual(self.candidates([self.detection(cls)], 4, shape, (12, 0, 0))[0][5], 4)
            self.assertEqual(self.candidates([self.detection(cls)], 7, shape, (80, -5, 65))[0][5], 7)
            self.assertEqual(self.candidates([self.detection(cls)], 2, shape), [])

    def test_model_mapping_retains_all_labels_and_allowed_class_set(self):
        self.assertEqual(len(yolo_config.CLASS_NAMES), 24)
        self.assertEqual(len(yolo_config.CLASS_SHAPES), 24)
        self.assertEqual(yolo_config.CLASS_NAMES[22], 'b奶娃')
        self.assertEqual({i for i, shape in enumerate(yolo_config.CLASS_SHAPES) if shape},
                         {0, 1, 2, 3, 4, 5, 10, 18, 19, 20, 21, 22, 23})
        self.assertNotIn(2, yolo_config.CLASS_SHAPES)

    def test_unmapped_shape_is_not_inferred_from_color(self):
        self.assertEqual(self.candidates([self.detection(24)], 1, 3), [])
        self.assertEqual(self.candidates([self.detection(19)], 1, 2), [])

    def test_actual_main_tracking_sends_matching_shape_and_invalid_on_loss(self):
        # Execute the production function with image/UART doubles, avoiding
        # sensor startup. This verifies the integration, not just packet helpers.
        tree = ast.parse((DEPLOY / 'main.py').read_text(encoding='utf-8'))
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in ('run_yolo_tracking', 'report_target_unavailable')]
        writes = []
        now = [0]
        canvas = types.SimpleNamespace(width=lambda: 640, height=lambda: 480,
            draw_rectangle=lambda *a, **kw: None, draw_cross=lambda *a, **kw: None,
            draw_string_advanced=lambda *a, **kw: None)
        env = dict(time=types.SimpleNamespace(ticks_ms=lambda: now[0], ticks_diff=lambda a, b: a-b),
            tracker=ColorTracker(confirm_frames=1), yolo=types.SimpleNamespace(source=(640, 480)),
            yolo_config=yolo_config, tracking_candidates=tracking_candidates,
            tracking_sequence=0, tracking_last_send=None, TRACK_SEND_PERIOD_MS=50,
            target_request=(61, 7, 4), target_tracking_packet=target_tracking_packet,
            tracking_packet=tracking_packet, uart=types.SimpleNamespace(write=writes.append))
        exec(compile(ast.Module(body=functions, type_ignores=[]), 'main_tracking', 'exec'), env)
        env['run_yolo_tracking'](self.frame(), [self.detection(3)], canvas, 35)
        self.assertEqual(writes[-1][:6], bytes((0xF6, 0, 1, 7, 4, 61)))
        now[0] = 50
        env['run_yolo_tracking'](self.frame(), [self.detection(19)], canvas, 35)
        self.assertEqual(writes[-1][:6], bytes((0xF6, 1, 0, 0, 0, 61)))
        now[0] = 100
        env['report_target_unavailable'](canvas)
        self.assertEqual(writes[-1][:6], bytes((0xF6, 2, 0, 0, 0, 61)))

    def test_actual_main_loop_accepts_target_request_and_sends_f6(self):
        class EndLoop(Exception):
            pass
        state = {'tick': 0, 'read': False, 'packets': [], 'closed': False}
        frame = self.frame()
        canvas = types.SimpleNamespace(width=lambda: 640, height=lambda: 480,
            draw_rectangle=lambda *a, **kw: None, draw_cross=lambda *a, **kw: None,
            draw_string_advanced=lambda *a, **kw: None)
        def tick():
            state['tick'] += 1
            if state['tick'] > 4:
                raise EndLoop()
        def read():
            state['read'] = True
            return request(42, 1, 3)
        uart = types.SimpleNamespace(any=lambda: not state['read'], read=read,
                                     write=lambda data: state['packets'].append(bytes(data)))
        sensor = types.SimpleNamespace(QVGA=(320, 240), QQVGA=(160, 120), VGA=(640, 480),
            snapshot=lambda: frame, snapshot_yolo=lambda: None,
            set_framesize=lambda *a: None, init=lambda **kw: None)
        detector = types.SimpleNamespace(source=(640, 480),
            detect=lambda unused: [self.detection(19)] if state['tick'] <= 3 else [],
            close=lambda: state.update(closed=True))
        modules = {
            'ustruct': __import__('struct'),
            'time': types.SimpleNamespace(clock=lambda: types.SimpleNamespace(tick=tick),
                ticks_ms=lambda: state['tick'] * 50, ticks_diff=lambda a, b: a-b),
            'ulab': types.SimpleNamespace(numpy=types.SimpleNamespace(array=lambda value: value,
                eye=lambda n: None, diag=lambda values: None)),
            'k230_hal': types.SimpleNamespace(sensor=sensor, create_uart=lambda: uart,
                set_light=lambda value: None, show_image=lambda image: None,
                preview_image=lambda: canvas, draw_text=lambda *a, **kw: None,
                shutdown=lambda *a: None),
            'vision_yolo': types.SimpleNamespace(YoloDetector=lambda *a, **kw: detector,
                tracking_candidates=tracking_candidates, draw_detections=lambda *a: None),
        }
        source = (DEPLOY / 'main.py').read_text(encoding='utf-8')
        with patch.dict(sys.modules, modules):
            with self.assertRaises(EndLoop):
                exec(compile(source, str(DEPLOY / 'main.py'), 'exec'), {})
        self.assertTrue(state['closed'])
        self.assertEqual(len(state['packets']), 5)
        self.assertEqual([p[0] for p in state['packets']], [0xF6] * 5)
        self.assertEqual([p[2] for p in state['packets']], [0, 0, 1, 0, 0])
        self.assertTrue(all(p[5] == 42 for p in state['packets']))
        self.assertEqual(state['packets'][2][3:5], bytes((1, 3)))


class CrossEndpointProtocolTests(unittest.TestCase):
    """Actual STM32 C encoder/parser against deployed K230 Python code."""
    @classmethod
    def setUpClass(cls):
        dll = ROOT / 'tests' / '.build' / 'vision_follow_host.dll'
        if not dll.exists():
            raise unittest.SkipTest('Build tests/build_host.cmd before cross-endpoint checks')
        cls.host = ctypes.CDLL(str(dll))
        cls.host.host_time.argtypes = [ctypes.c_uint32]
        cls.host.host_byte.argtypes = [ctypes.c_uint8]
        cls.host.host_target_start.argtypes = [ctypes.c_int, ctypes.c_int]

    def begin(self, color, shape):
        self.host.host_reset()
        self.host.host_time(100)
        self.assertEqual(self.host.host_target_start(color, shape), 1)
        wire = bytes(self.host.host_command(index) for index in range(6))
        parser = TargetCommandParser()
        self.assertEqual(parser.feed(wire[:2], 100), [])
        events = parser.feed(wire[2:], 101)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], 'target')
        self.assertEqual(events[0][2:], (color, shape))
        return events[0][1]

    def send(self, wire):
        for byte in wire:
            self.host.host_byte(byte)

    def test_all_supported_pairs_roundtrip_through_actual_c_and_python(self):
        for color in (1, 2, 3, 4, 7):
            for shape in (1, 3, 4, 5, 6):
                if not target_supported(color, shape):
                    continue
                with self.subTest(color=color, shape=shape):
                    token = self.begin(color, shape)
                    wire = target_tracking_packet(19, token, shape, (1, color, 165, 133, 40, 50))
                    self.send(wire[:7])
                    self.assertEqual(self.host.host_observe_read(), 0)
                    self.send(wire[7:])
                    self.assertEqual(self.host.host_observe_read(), 1)
                    self.assertEqual(self.host.host_target_color(), color)
                    self.assertEqual(self.host.host_target_shape(), shape)
                    self.assertEqual((self.host.host_x(), self.host.host_y()), (165, 133))
                    self.assertEqual(self.host.host_calls(), 0)

    def test_python_packets_with_wrong_token_shape_and_legacy_header_rejected(self):
        token = self.begin(4, 1)
        wrong = (
            target_tracking_packet(1, token + 1, 1, (1, 4, 165, 133, 40, 50)),
            target_tracking_packet(2, token, 3, (1, 4, 165, 133, 40, 50)),
            tracking_packet(3, (1, 1, 165, 133, 40, 50)),
        )
        for wire in wrong:
            self.send(wire)
            self.assertEqual(self.host.host_observe_read(), 0)
        self.send(target_tracking_packet(4, token, 1, (1, 4, 165, 133, 40, 50)))
        self.assertEqual(self.host.host_observe_read(), 1)
        self.send(target_tracking_packet(5, token, 1, (0, 4, 165, 133, 40, 50)))
        self.assertEqual(self.host.host_observe_read(), 0)

    def test_python_packet_wrong_color_rejected_even_with_correct_shape(self):
        token = self.begin(1, 5)
        self.send(target_tracking_packet(1, token, 5, (1, 2, 165, 133, 40, 50)))
        self.assertEqual(self.host.host_observe_read(), 0)
        self.send(target_tracking_packet(2, token, 5, (1, 1, 165, 133, 40, 50)))
        self.assertEqual(self.host.host_observe_read(), 1)


if __name__ == '__main__':
    unittest.main()
