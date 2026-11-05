"""Seven LAB color profiles for material preview, independent of motor commands.
Thresholds are starting values; calibrate under the actual lighting.
"""
# name, LAB threshold, outline RGB, reference LAB (for overlapping colors)
COLOR_PROFILES = (
    ('红色', (15, 85, 25, 90, 0, 70), (255, 60, 60), (45, 55, 30)),
    ('绿色', (15, 90, -80, -15, -5, 75), (50, 255, 50), (55, -45, 30)),
    ('蓝色', (10, 65, -5, 60, -90, -22), (65, 110, 255), (35, 25, -55)),
    ('黑色', (0, 25, -18, 18, -18, 18), (255, 255, 255), (12, 0, 0)),
    ('橙色', (25, 90, 10, 55, 30, 90), (255, 155, 30), (60, 30, 60)),
    ('浅蓝', (35, 100, -55, 2, -65, -5), (60, 230, 255), (70, -22, -30)),
    ('黄色', (45, 100, -25, 15, 30, 100), (255, 230, 30), (80, -5, 65)),
)
MIN_PIXELS = 200
MIN_AREA = 200
MAX_BOXES = 16


def intersection_over_union(first, second):
    x, y, w, h = first
    sx, sy, sw, sh = second
    iw = max(0, min(x + w, sx + sw) - max(x, sx))
    ih = max(0, min(y + h, sy + sh) - max(y, sy))
    overlap = iw * ih
    return overlap / max(1, w * h + sw * sh - overlap)


def detect_materials(img):
    candidates = []
    frame_area = img.width() * img.height()
    # Read the untouched detection image; draw only on the preview canvas.
    for index, profile in enumerate(COLOR_PROFILES):
        name, threshold, color, reference = profile
        blobs = img.find_blobs([threshold], pixels_threshold=MIN_PIXELS,
                              area_threshold=MIN_AREA, merge=True, margin=3)
        for blob in blobs:
            rect = blob.rect()
            area = rect[2] * rect[3]
            if area > frame_area * 0.65 or blob.pixels() / max(1, area) < 0.25:
                continue
            stats = img.get_statistics(roi=rect)
            score = ((stats.a_mean() - reference[1]) ** 2 +
                     (stats.b_mean() - reference[2]) ** 2 +
                     0.2 * (stats.l_mean() - reference[0]) ** 2)
            candidates.append((score, index, rect, blob.cx(), blob.cy()))
    candidates.sort(key=lambda item: item[0])
    results = []
    for candidate in candidates:
        if any(intersection_over_union(candidate[2], previous[2]) > 0.45
               for previous in results):
            continue
        results.append(candidate)
        if len(results) >= MAX_BOXES:
            break
    return results
