"""YOLO11/YOLOv8 decoded detection output, K230 KPU + AI2D.

Supported output: [1, 4+nc, N] or [1, N, 4+nc], xywh in input pixels.
Raw detection heads, segmentation, pose and embedded NMS are rejected.
"""
import math


def geometry(source, target, mode):
    sw, sh = source
    tw, th = target
    if mode == 'stretch':
        return tw / sw, th / sh, 0, 0, 0, 0
    if mode not in ('letterbox', 'letterbox_top_left'):
        raise ValueError('Unknown YOLO preprocessing: ' + mode)
    scale = min(tw / sw, th / sh)
    if mode == 'letterbox_top_left':
        # Match the board's libs.Utils.letterbox_pad_param: truncate the
        # resized dimensions and place all padding on the bottom/right.
        rw, rh = int(sw * scale), int(sh * scale)
        return rw / sw, rh / sh, 0, 0, tw - rw, th - rh
    rw, rh = int(round(sw * scale)), int(round(sh * scale))
    left, top = (tw - rw) // 2, (th - rh) // 2
    return rw / sw, rh / sh, left, top, tw - rw - left, th - rh - top


def iou(a, b):
    x, y, w, h = a
    bx, by, bw, bh = b
    overlap = max(0, min(x+w, bx+bw)-max(x, bx)) * max(0, min(y+h, by+bh)-max(y, by))
    return overlap / max(1, w*h + bw*bh - overlap)


def decode(output, source, target, nc, confidence, nms_iou,
           max_candidates=100, max_detections=16, mode='letterbox'):
    shape = output.shape
    if len(shape) != 3 or shape[0] != 1:
        raise ValueError('Expected decoded YOLO [1,C,N], got ' + str(shape))
    channels = 4 + nc
    if shape[1] == channels and shape[2] != channels:
        count, transposed = shape[2], False
    elif shape[2] == channels and shape[1] != channels:
        count, transposed = shape[1], True
    else:
        raise ValueError('YOLO class count/layout mismatch: ' + str(shape))
    sx, sy, left, top, _, _ = geometry(source, target, mode)
    candidates = []
    # Bounded candidate list keeps Python NMS costs predictable.
    for index in range(count):
        best, cls = 0.0, 0
        for c in range(nc):
            score = float(output[0, index, c+4] if transposed else output[0, c+4, index])
            if score > best:
                best, cls = score, c
        if not confidence <= best <= 1.0:
            continue
        values = [float(output[0, index, j] if transposed else output[0, j, index]) for j in range(4)]
        if not all(math.isfinite(v) for v in values):
            continue
        cx, cy, w, h = values
        if w <= 0 or h <= 0:
            continue
        x1 = max(0, min(source[0], (cx-w/2-left)/sx))
        y1 = max(0, min(source[1], (cy-h/2-top)/sy))
        x2 = max(0, min(source[0], (cx+w/2-left)/sx))
        y2 = max(0, min(source[1], (cy+h/2-top)/sy))
        rect = (int(x1), int(y1), int(x2)-int(x1), int(y2)-int(y1))
        if rect[2] < 2 or rect[3] < 2:
            continue
        item = (cls, best, rect)
        if len(candidates) >= max_candidates:
            if best <= candidates[-1][1]:
                continue
            candidates.pop()
        candidates.append(item)
        candidates.sort(key=lambda d: d[1], reverse=True)
    result = []
    for item in candidates:
        if any(item[0] == old[0] and iou(item[2], old[2]) > nms_iou for old in result):
            continue
        result.append(item)
        if len(result) >= max_detections:
            break
    return result


def class_name(cls, names):
    return names[cls] if cls < len(names) else 'class_%d' % cls


def tracking_candidates(detections, img, source, wanted, class_colors=(),
                        class_shapes=(), wanted_shape=0):
    # Coordinates are QVGA pixels; color/shape IDs are not YOLO class indexes.
    # Zero keeps legacy color-only tracking. Task requests require a known
    # YOLO shape; a colored blob or an unknown label can never supply its shape.
    from vision_colors import COLOR_PROFILES
    candidates = []
    sx, sy = img.width()/source[0], img.height()/source[1]
    for cls, score, box in detections:
        if cls < 0:
            continue
        if wanted_shape and (cls >= len(class_shapes) or
                             class_shapes[cls] != wanted_shape):
            continue
        x, y, w, h = box
        rect = (int(x*sx), int(y*sy), max(1, int(w*sx)), max(1, int(h*sy)))
        color = class_colors[cls] if cls < len(class_colors) else 0
        if color == 0:
            # For shape-only models classify the interior of each YOLO object.
            rx, ry, rw, rh = rect
            inset = (rx+rw//4, ry+rh//4, max(1,rw//2), max(1,rh//2))
            stats = img.get_statistics(roi=inset)
            lab = (stats.l_mean(), stats.a_mean(), stats.b_mean())
            ranked = []
            for index, profile in enumerate(COLOR_PROFILES):
                t, ref = profile[1], profile[3]
                if all(t[j*2] <= lab[j] <= t[j*2+1] for j in range(3)):
                    distance = 0.2*(lab[0]-ref[0])**2 + (lab[1]-ref[1])**2 + (lab[2]-ref[2])**2
                    ranked.append((distance, index+1))
            if ranked:
                ranked.sort()
                color = ranked[0][1]
        if color not in wanted:
            continue
        x, y, w, h = rect
        candidates.append((x+w//2, y+h//2, w, h, w*h, color, rect))
    return candidates


def draw_detections(canvas, detections, source, names):
    sx, sy = canvas.width()/source[0], canvas.height()/source[1]
    for cls, score, rect in detections:
        x, y, w, h = rect
        x, y, w, h = int(x*sx), int(y*sy), int(w*sx), int(h*sy)
        color = (50, 255, 80)
        canvas.draw_rectangle((x,y,w,h), color=color, thickness=2)
        canvas.draw_cross(x+w//2, y+h//2, color=color, size=8)
        canvas.draw_string_advanced(x, max(0,y-24), 22,
                                   '%s %.2f' % (class_name(cls,names),score), color=color)
    if not detections:
        canvas.draw_string_advanced(4,4,24,'YOLO: 未检测到目标',color=(255,255,255))


class YoloDetector:
    def __init__(self, config, source=(640,480)):
        import nncase_runtime as nn
        from ulab import numpy as np
        self.config, self.source = config, source
        self.kpu = nn.kpu()
        self.ai2d = self.builder = self.output_tensor = self.output_array = None
        self.kpu.load_kmodel(config.MODEL_PATH)
        iw, ih = config.INPUT_SIZE
        desc = self.kpu.inputs_desc(0)
        if self.kpu.inputs_size() != 1 or desc['tensor_size'] != 3*iw*ih or desc['tensor_datatype'] != 'dt_uint8':
            raise ValueError('Model requires different input size/type; check yolo_config.py')
        if self.kpu.outputs_size() != 1:
            raise ValueError('Only decoded single-output YOLO detection is supported')
        if config.CLASS_NAMES and len(config.CLASS_NAMES) != config.NUM_CLASSES:
            raise ValueError('CLASS_NAMES length must equal NUM_CLASSES')
        self.ai2d = nn.ai2d()
        self.ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        _, _, left, top, right, bottom = geometry(source, config.INPUT_SIZE, config.PREPROCESS)
        if config.PREPROCESS in ('letterbox', 'letterbox_top_left'):
            self.ai2d.set_pad_param(True,[0,0,0,0,top,bottom,left,right],0,[config.PAD_VALUE]*3)
        self.ai2d.set_resize_param(True,nn.interp_method.tf_bilinear,nn.interp_mode.half_pixel)
        self.output_array = np.ones((1,3,ih,iw),dtype=np.uint8)
        self.output_tensor = nn.from_numpy(self.output_array)
        self.builder = self.ai2d.build([1,3,source[1],source[0]],[1,3,ih,iw])
        print('YOLO loaded:',config.MODEL_PATH,config.INPUT_SIZE,'classes:',config.NUM_CLASSES)

    def detect(self, img):
        import nncase_runtime as nn
        if (img.width(),img.height()) != self.source:
            raise ValueError('YOLO source resolution mismatch')
        input_tensor = nn.from_numpy(img.to_numpy_ref())
        self.builder.run(input_tensor,self.output_tensor)
        self.kpu.set_input_tensor(0,self.output_tensor)
        self.kpu.run()
        output_tensor = self.kpu.get_output_tensor(0)
        output = output_tensor.to_numpy()
        c = self.config
        result = decode(output,self.source,c.INPUT_SIZE,c.NUM_CLASSES,c.CONFIDENCE,
                        c.NMS_IOU,c.MAX_CANDIDATES,c.MAX_DETECTIONS,c.PREPROCESS)
        del output, output_tensor, input_tensor
        return result

    def close(self):
        self.builder = self.output_tensor = self.output_array = self.ai2d = self.kpu = None
        import gc
        gc.collect()
