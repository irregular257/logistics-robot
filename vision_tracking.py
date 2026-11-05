"""Small, testable tracker and wire protocol shared by OpenMV tracking mode."""

TRACK_HEADER = 0xF9
TRACK_TAIL = 0xFE
TRACK_FRAME_BYTES = 14
TARGET_REQUEST_HEADER = 0xF7
TARGET_TRACK_HEADER = 0xF6
TARGET_FRAME_BYTES = 16
TARGET_COLORS = (1, 2, 3, 4, 7)  # red, green, blue, black, yellow
TARGET_SHAPES = (1, 3, 4, 5, 6)  # hexagonal prism, cylinder, pentagonal prism, cuboid, 奶娃


def target_supported(color, shape):
    """Only combinations present in the current model may become tasks."""
    if shape == 1:
        return color == 4
    if shape == 3:
        return color in (1, 2, 3, 4)
    if shape == 4:
        return color in (1, 2, 7)
    return shape in (5, 6) and color in TARGET_COLORS


def crc8(data):
    """CRC-8/ATM, polynomial 0x07, initial value 0."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def tracking_packet(sequence, result):
    """Encode uint16 coordinates; absent/unconfirmed targets always send zeros.

    result: (valid, color, cx, cy, width, height).
    """
    valid, color, x, y, width, height = result
    if not valid:
        color = x = y = width = height = 0
    frame = bytearray((TRACK_HEADER, sequence & 0xFF, int(bool(valid)), color))
    for value in (x, y, width, height):
        value = int(value)
        frame.append(value & 0xFF)
        frame.append((value >> 8) & 0xFF)
    frame.append(crc8(frame))
    frame.append(TRACK_TAIL)
    return frame


def target_tracking_packet(sequence, request_token, shape, result):
    """A color+shape observation bound to one STM32 selection request.

    Missing/unconfirmed targets keep the request token but clear target data.
    The receiver must reject a different token, color or shape before moving.
    """
    valid, color, x, y, width, height = result
    valid = bool(valid) and target_supported(color, shape)
    if not valid:
        color = shape = x = y = width = height = 0
    frame = bytearray((TARGET_TRACK_HEADER, sequence & 0xFF, int(valid),
                       color, shape, request_token & 0xFF))
    for value in (x, y, width, height):
        value = int(value)
        frame.append(value & 0xFF)
        frame.append((value >> 8) & 0xFF)
    frame.append(crc8(frame))
    frame.append(TRACK_TAIL)
    return frame


class TargetCommandParser:
    """Demultiplex framed target selections and legacy one-byte commands.

    Payload bytes must never become legacy commands (color IDs overlap old
    movement commands). Bad/truncated frames quarantine the stream until a
    new valid frame or explicit stop. A possible nested header is retained
    for recovery without replaying payload bytes as legacy commands.
    """
    def __init__(self, ticks_diff=None, gap_ms=1000):
        self.ticks_diff = ticks_diff or (lambda now, then: now - then)
        self.gap_ms = gap_ms
        self.buffer = bytearray()
        self.last_byte = None
        self.quarantined = False

    def feed(self, data, now_ms):
        events = []
        if self.buffer and self.ticks_diff(now_ms, self.last_byte) >= self.gap_ms:
            self.buffer = bytearray()
            self.quarantined = True
        for byte in data:
            self.last_byte = now_ms
            if not self.buffer:
                if byte == TARGET_REQUEST_HEADER:
                    self.buffer.append(byte)
                elif byte == 30:
                    self.quarantined = False
                    events.append(('legacy', byte))
                elif not self.quarantined:
                    events.append(('legacy', byte))
                continue
            self.buffer.append(byte)
            if len(self.buffer) == 6:
                frame = self.buffer
                self.buffer = bytearray()
                if (frame[5] == TRACK_TAIL and crc8(frame[:4]) == frame[4]
                        and target_supported(frame[2], frame[3])):
                    self.quarantined = False
                    events.append(('target', frame[1], frame[2], frame[3]))
                else:
                    self.quarantined = True
                    for index in range(1, len(frame)):
                        if frame[index] == TARGET_REQUEST_HEADER:
                            self.buffer = bytearray(frame[index:])
                            break
        return events


class ColorTracker:
    """Lock one colored blob using position, size and short-term velocity.

    Candidates are (cx, cy, width, height, pixels, color[, bbox]), in QVGA pixels.
    No predicted position is ever advertised as an observed valid target.
    """

    def __init__(self, ticks_diff=None, alpha=0.45, reacquire_ms=500, confirm_frames=3):
        self.ticks_diff = ticks_diff or (lambda now, then: now - then)
        self.alpha = alpha
        self.reacquire_ms = reacquire_ms
        self.confirm_frames = confirm_frames
        self.reset()

    def reset(self):
        self.locked = False
        self.x = self.y = 0.0
        self.width = self.height = 0.0
        self.vx = self.vy = 0.0
        self.color = 0
        self.last_seen = None
        self.hits = 0
        self.observation = None  # Raw matched blob; cleared on every missed frame.

    def update(self, candidates, now_ms):
        if self.locked and self.ticks_diff(now_ms, self.last_seen) >= self.reacquire_ms:
            self.reset()

        chosen = None
        if not self.locked:
            if candidates:
                chosen = max(candidates, key=lambda item: item[4])
        else:
            elapsed = max(1, self.ticks_diff(now_ms, self.last_seen))
            prediction_dt = min(elapsed, 150) / 1000.0
            px = self.x + self.vx * prediction_dt
            py = self.y + self.vy * prediction_dt
            gate = min(90.0, 45.0 + elapsed * 0.15)
            old_area = max(1.0, self.width * self.height)
            best_score = None
            for item in candidates:
                x, y, width, height, pixels, color = item[:6]
                ratio = width * height / old_area
                distance2 = (x - px) ** 2 + (y - py) ** 2
                if color != self.color or not 0.35 <= ratio <= 2.8 or distance2 > gate * gate:
                    continue
                score = distance2 / (gate * gate) + 0.15 * abs(ratio - 1.0)
                if best_score is None or score < best_score:
                    best_score = score
                    chosen = item

        if chosen is None:
            self.hits = 0
            self.observation = None
            return (0, 0, 0, 0, 0, 0)

        self.observation = chosen
        x, y, width, height, pixels, color = chosen[:6]
        if not self.locked:
            self.x, self.y = float(x), float(y)
            self.width, self.height = float(width), float(height)
            self.vx = self.vy = 0.0
            self.locked = True
        else:
            dt = max(1, self.ticks_diff(now_ms, self.last_seen)) / 1000.0
            old_x, old_y = self.x, self.y
            self.x += self.alpha * (x - self.x)
            self.y += self.alpha * (y - self.y)
            self.width += self.alpha * (width - self.width)
            self.height += self.alpha * (height - self.height)
            self.vx = max(-300.0, min(300.0, (self.x - old_x) / dt))
            self.vy = max(-300.0, min(300.0, (self.y - old_y) / dt))
        self.color = color
        self.last_seen = now_ms
        self.hits += 1
        valid = self.hits >= self.confirm_frames
        return (int(valid), color, int(self.x + 0.5), int(self.y + 0.5),
                int(self.width + 0.5), int(self.height + 0.5))
