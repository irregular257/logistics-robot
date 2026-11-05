"""Edit after calibration; distances in mm, widths in QVGA pixels.

Index 0 is unused; indices 1/2/3 are red/green/blue.
For each color, place the SAME material front-facing at a measured optical-axis
depth, and read W=...px in the CanMV preview. Enter that distance and width.
Example ONLY: 300 mm and 60 px -> K = 18000 mm*px -> 30 px means 600 mm.
Changing lens, resolution, camera pose or material size requires recalibration.
"""

REFERENCE_DISTANCE_MM = (0, 0, 0, 0)
REFERENCE_WIDTH_PX = (0, 0, 0, 0)
CALIBRATION_FRAME_SIZE = (320, 240)

MIN_WIDTH_PX = 8
BORDER_MARGIN_PX = 2
MIN_DISTANCE_MM = 30
MAX_DISTANCE_MM = 3000
RANGE_ALPHA = 0.45
CALIBRATION_LOG = False  # Print raw width and range at the UART send rate.
