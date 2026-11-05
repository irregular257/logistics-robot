"""Calibrated apparent-width depth estimation, independent of OpenMV APIs.

Z = reference_depth_mm * reference_width_px / current_width_px.
This is optical-axis depth, not floor distance or guaranteed obstacle clearance.
Only a fully visible, known-size target with the calibrated pose is supported.
"""

from vision_tracking import crc8

RANGE_HEADER = 0xF8
RANGE_FRAME_BYTES = 9
RANGE_OK = 0
RANGE_NO_TARGET = 1
RANGE_UNCALIBRATED = 2
RANGE_BAD_SIZE = 3
RANGE_OUT_OF_BOUNDS = 4
RANGE_BAD_RESOLUTION = 5


def ranging_packet(sequence, color, result):
    """F8, seq, valid, color, mm_lo, mm_hi, reason, CRC8(first7), FE."""
    valid, distance_mm, reason = result
    distance_mm = int(distance_mm) if valid else 0
    if valid and (reason != RANGE_OK or not 1 <= distance_mm <= 65535):
        raise ValueError("Invalid distance or status")
    frame = bytearray((RANGE_HEADER, sequence & 0xFF, int(bool(valid)), color,
                       distance_mm & 0xFF, (distance_mm >> 8) & 0xFF, reason))
    frame.append(crc8(frame))
    frame.append(0xFE)
    return frame


class VisualRanger:
    def __init__(self, reference_distances, reference_widths,
                 frame_size=(320, 240), min_width=8, border_margin=2,
                 min_distance=30, max_distance=3000, alpha=0.45):
        self.references = reference_distances
        self.widths = reference_widths
        self.frame_size = frame_size
        self.min_width = min_width
        self.border_margin = border_margin
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.alpha = alpha
        self.reset()

    def reset(self):
        self.filtered = None
        self.color = 0

    def _invalid(self, reason):
        self.reset()
        return (0, 0, reason)

    def measure(self, confirmed, observation, frame_size):
        """Use the tracker's current RAW blob and bbox; never a prediction.

        observation: (cx, cy, w, h, pixels, color, (left, top, w, h)).
        The bbox must be from blob.rect(), not reconstructed from its centroid.
        """
        if not confirmed or observation is None:
            return self._invalid(RANGE_NO_TARGET)
        if frame_size != self.frame_size:
            return self._invalid(RANGE_BAD_RESOLUTION)
        cx, cy, width, height, pixels, color, rect = observation
        if not 1 <= color <= 3 or color >= len(self.references) or color >= len(self.widths):
            return self._invalid(RANGE_UNCALIBRATED)
        ref_distance = self.references[color]
        ref_width = self.widths[color]
        # Comparisons also reject NaN and infinity without math.isfinite(),
        # which is not present in all OpenMV MicroPython builds.
        if not (0 < ref_distance <= 65535 and self.min_width <= ref_width <= frame_size[0]):
            return self._invalid(RANGE_UNCALIBRATED)
        left, top, box_width, box_height = rect
        margin = self.border_margin
        if (width < self.min_width or height <= 0 or box_width != width or box_height != height or
                left <= margin or top <= margin or
                left + width >= frame_size[0] - margin or
                top + height >= frame_size[1] - margin):
            return self._invalid(RANGE_BAD_SIZE)
        distance = ref_distance * ref_width / width
        if not self.min_distance <= distance <= min(self.max_distance, 65535):
            return self._invalid(RANGE_OUT_OF_BOUNDS)
        if self.filtered is None or self.color != color:
            self.filtered = distance
        else:
            self.filtered += self.alpha * (distance - self.filtered)
        self.color = color
        return (1, int(self.filtered + 0.5), RANGE_OK)
