"""
Auto tighten LAB thresholds for red/green/blue with passive observation.

Flash this script to the OpenMV, place the camera watching the field, then let
each colored target pass through the ROI. Every color is expected to stay in
view for about 4 s; the script automatically detects which color is present,
collects statistics for that window, and tightens the thresholds accordingly.
Once all three colors have been processed it freezes the current sensor gain /
white balance / exposure and prints the final threshold table.
"""

import math
import sensor
import image
import time
from pyb import Pin, Timer

# Keep camera configuration identical to the main vision program.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True, gain_db_ceiling=16.0)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True, exposure_us=5000)
sensor.set_hmirror(False)
sensor.set_vflip(False)
sensor.set_windowing((0, 0, 320, 240))

BLOCK_ROI = (70, 30, 160, 160)
COLOR_NAMES = ("red", "green", "blue")


DEFAULT_THRESHOLDS = {
    "red": (15, 90, 15, 60, -10, 60),
    "green": (35, 95, -85, -30, -25, 50),
    "blue": (10, 75, -20, 35, -120, -15),
}



LAB_LIMITS = {
    "L": (0, 100),
    "A": (-128, 127),
    "B": (-128, 127),
}

TIGHTEN_WINDOW_MS = 4000
MIN_TOTAL_SAMPLES = 40          # ignore very short detections
CORE_SAMPLE_COUNT = 30          # keep the most stable 30 frames
EDGE_GUARD_FRAMES = 6           # drop this many frames from both ends before centering
SIGMA_SCALE = 1.2
MIN_SPAN = {"L": 12, "A": 10, "B": 12}
IDLE_RESET_MS = 400            # if color disappears for this long without enough data, discard

DEBUG_MODE = True  # set False to silence debug prints
LIGHT_ENABLED = True  # enable fill light by default
LIGHT_ON_AT_START = True  # turn light on immediately after init

tim = Timer(2, freq=500)
half_period = int(tim.period() * 0.45)
pwm_channel = tim.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width=0)
if LIGHT_ENABLED and LIGHT_ON_AT_START:
    pwm_channel.pulse_width(half_period)


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value

def ensure_min_span(low, high, span_min, limits):
    if (high - low) >= span_min:
        return low, high
    midpoint = (low + high) / 2.0
    half = span_min / 2.0
    low = clamp(int(round(midpoint - half)), *limits)
    high = clamp(int(round(midpoint + half)), *limits)
    return low, high


def mean_std(sum_val, sum_sq_val, count):
    mean_val = sum_val / count
    variance = max((sum_sq_val / count) - (mean_val * mean_val), 0.0)
    return mean_val, math.sqrt(variance)


class TightenState:
    def __init__(self):
        self.states = {
            color: {
                "done": False,
                "collecting": False,
                "start": 0,
                "last_seen": 0,
                "count": 0,
                "samples": [],
            } for color in COLOR_NAMES
        }
        self.thresholds = {color: DEFAULT_THRESHOLDS[color] for color in COLOR_NAMES}
        self.locked = False

    def active(self, color):
        return self.states[color]["collecting"]

    def start(self, color):
        info = self.states[color]
        info.update({
            "collecting": True,
            "start": time.ticks_ms(),
            "last_seen": time.ticks_ms(),
            "count": 0,
            "samples": [],
        })
        if DEBUG_MODE:
            print("TIGHTEN start:", color.upper())

    def accumulate(self, color, stats):
        info = self.states[color]
        l_val = int(stats.l_mean())
        a_val = int(stats.a_mean())
        b_val = int(stats.b_mean())
        info["samples"].append((l_val, a_val, b_val))
        info["count"] = len(info["samples"])
        info["last_seen"] = time.ticks_ms()

    def finalize(self, color):
        info = self.states[color]
        if info["done"] or not info["collecting"]:
            return

        samples = info["samples"]
        total_count = len(samples)
        if total_count < MIN_TOTAL_SAMPLES:
            if DEBUG_MODE:
                print("TIGHTEN {} skipped (only {} samples)".format(color.upper(), total_count))
            info["collecting"] = False
            info["samples"] = []
            info["count"] = 0
            return

        usable = samples[:]
        if total_count > EDGE_GUARD_FRAMES * 2:
            usable = samples[EDGE_GUARD_FRAMES:-EDGE_GUARD_FRAMES]

        if len(usable) >= CORE_SAMPLE_COUNT:
            start = max((len(usable) - CORE_SAMPLE_COUNT) // 2, 0)
            trimmed = usable[start:start + CORE_SAMPLE_COUNT]
        else:
            trimmed = usable

        l_sum = sum(item[0] for item in trimmed)
        a_sum = sum(item[1] for item in trimmed)
        b_sum = sum(item[2] for item in trimmed)
        l_sq_sum = sum(item[0] * item[0] for item in trimmed)
        a_sq_sum = sum(item[1] * item[1] for item in trimmed)
        b_sq_sum = sum(item[2] * item[2] for item in trimmed)
        count = len(trimmed)

        l_mean, l_std = mean_std(l_sum, l_sq_sum, count)
        a_mean, a_std = mean_std(a_sum, a_sq_sum, count)
        b_mean, b_std = mean_std(b_sum, b_sq_sum, count)

        l_low = clamp(int(round(l_mean - SIGMA_SCALE * l_std)), *LAB_LIMITS["L"])
        l_high = clamp(int(round(l_mean + SIGMA_SCALE * l_std)), *LAB_LIMITS["L"])
        a_low = clamp(int(round(a_mean - SIGMA_SCALE * a_std)), *LAB_LIMITS["A"])
        a_high = clamp(int(round(a_mean + SIGMA_SCALE * a_std)), *LAB_LIMITS["A"])
        b_low = clamp(int(round(b_mean - SIGMA_SCALE * b_std)), *LAB_LIMITS["B"])
        b_high = clamp(int(round(b_mean + SIGMA_SCALE * b_std)), *LAB_LIMITS["B"])

        l_low, l_high = ensure_min_span(l_low, l_high, MIN_SPAN["L"], LAB_LIMITS["L"])
        a_low, a_high = ensure_min_span(a_low, a_high, MIN_SPAN["A"], LAB_LIMITS["A"])
        b_low, b_high = ensure_min_span(b_low, b_high, MIN_SPAN["B"], LAB_LIMITS["B"])

        self.thresholds[color] = (l_low, l_high, a_low, a_high, b_low, b_high)
        info["collecting"] = False
        info["done"] = True
        info["samples"] = []
        info["count"] = 0

        print("TIGHTEN {} -> {}".format(color.upper(), self.thresholds[color]))
        if DEBUG_MODE:
            print("   mean=({:.1f},{:.1f},{:.1f}) std=({:.2f},{:.2f},{:.2f}) count={}".format(
                l_mean, a_mean, b_mean, l_std, a_std, b_std, count))
            print("   total_samples={} used_middle={}".format(total_count, count))

    def all_done(self):
        return all(self.states[color]["done"] for color in COLOR_NAMES)

    def finalize_due(self):
        now = time.ticks_ms()
        for color in COLOR_NAMES:
            info = self.states[color]
            if info["collecting"]:
                idle = time.ticks_diff(now, info["last_seen"])
                if info["count"] >= MIN_TOTAL_SAMPLES and idle > IDLE_RESET_MS:
                    self.finalize(color)
                elif info["count"] < MIN_TOTAL_SAMPLES and idle > IDLE_RESET_MS:
                    if DEBUG_MODE:
                        print("TIGHTEN {} discarded (idle, {} frames)".format(color.upper(), info["count"]))
                    info["collecting"] = False
                    info["samples"] = []
                    info["count"] = 0


state = TightenState()


def freeze_sensor_settings():
    try:
        gain_db = sensor.get_gain_db()
    except Exception:
        gain_db = None
    try:
        rgb_gain_db = sensor.get_rgb_gain_db()
    except Exception:
        rgb_gain_db = None
    try:
        exposure_us = sensor.get_exposure_us()
    except Exception:
        exposure_us = None

    if gain_db is None:
        sensor.set_auto_gain(False)
    else:
        sensor.set_auto_gain(False, gain_db=gain_db)

    if rgb_gain_db is None:
        sensor.set_auto_whitebal(False)
    else:
        sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain_db)

    if exposure_us is None:
        sensor.set_auto_exposure(False)
    else:
        sensor.set_auto_exposure(False, exposure_us=exposure_us)

    print("SENSOR LOCKED gain={} rgb_gain={} exposure={}".format(gain_db, rgb_gain_db, exposure_us))


print("Auto tighten calibrator ready.")
print("Present red, green, and blue targets in the ROI for roughly 4 seconds each.")

while True:
    img = sensor.snapshot()

    for color in COLOR_NAMES:
        if state.states[color]["done"]:
            continue

        blobs = img.find_blobs([DEFAULT_THRESHOLDS[color]],
                               roi=BLOCK_ROI,
                               pixels_threshold=600,
                               area_threshold=400,
                               merge=True,
                               margin=15)
        if not blobs:
            continue

        blob = max(blobs, key=lambda item: item.pixels())
        stats = img.get_statistics(roi=(blob.x(), blob.y(), blob.w(), blob.h()))

        if not state.active(color):
            state.start(color)

        state.accumulate(color, stats)

        if DEBUG_MODE and state.states[color]["count"] % 10 == 0:
            sample_count = state.states[color]["count"]
            print("{} collected {} samples (latest L={} A={} B={})".format(
                color.upper(), sample_count,
                int(stats.l_mean()),
                int(stats.a_mean()),
                int(stats.b_mean()),
            ))

    state.finalize_due()

    if state.all_done():
        if not state.locked:
            freeze_sensor_settings()
            state.locked = True

            print("")
            print("=== FINAL TIGHTENED THRESHOLDS ===")
            for color in COLOR_NAMES:
                print("{}: {}".format(color.upper(), state.thresholds[color]))

            print("")
            print("thresholds = [")
            for color in COLOR_NAMES:
                print("    {},  # {}".format(state.thresholds[color], color.upper()))
            print("]")

            print("")
            print("Copy the list above into vision.py. Script keeps running; press reset to tighten again.")
        time.sleep_ms(1000)
    else:
        time.sleep_ms(30)
