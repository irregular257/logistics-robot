"""YOLO11 export from 14634; preserve the exported class order."""
ENABLED = True
MODEL_PATH = '/sdcard/yolo11n_det_320.kmodel'
INPUT_SIZE = (320, 320)  # uint8 RGB NCHW
NUM_CLASSES = 24
CLASS_NAMES = (
    '黑色六棱柱',
    '黑色圆柱',
    '红色五棱柱',
    '黄色五棱柱',
    '绿色五棱柱',
    '绿色圆柱',
    '浅蓝色三棱锥',
    '浅蓝色五棱柱',
    '浅蓝色圆柱',
    '三棱锥',
    '长方体',
    'a黑色正方体',
    'a红色正方体',
    'a黄色正方体',
    'a绿色正方体',
    'a浅蓝色正方体',
    'a深蓝色正方体',
    'b橙色圆柱',
    'b黑色圆柱',
    'b红色圆柱',
    'b蓝色圆柱',
    'b绿色圆柱',
    'b奶娃',
    'b深蓝色圆柱',
 )
# 0=LAB in ROI, 1=red, 2=green, 3=blue, 4=black,
# 5=orange, 6=light blue, 7=yellow. Unknown label b奶娃 uses LAB.
# Legacy tracking accepts RGB; task tracking additionally accepts black/yellow.
CLASS_COLORS = (4, 4, 1, 7, 2, 2, 6, 6, 6, 0, 0, 4, 1, 7, 2, 6, 3, 5, 4, 1, 3, 2, 0, 3)
# Task shape IDs: 0=excluded, 1=hexagonal prism, 3=cylinder,
# 4=pentagonal prism, 5=cuboid, 6=奶娃 (the original "b奶娃" class).
# User excludes all cubes, pyramids, light-blue and orange classes from tasks.
# All 24 labels remain available for inference/preview; preserve output order.
CLASS_SHAPES = (1, 3, 4, 4, 4, 3, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 6, 3)
CONFIDENCE = 0.6
NMS_IOU = 0.45
MAX_CANDIDATES = 100
MAX_DETECTIONS = 16
# Match the installed libs.YOLO.YOLO11 demo: top-left image, bottom/right pad.
PREPROCESS = 'letterbox_top_left'
PAD_VALUE = 128
