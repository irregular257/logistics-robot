import sys
import types
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1] / 'K230部署'))
from vision_yolo import decode, geometry, tracking_candidates, class_name

class YoloTests(unittest.TestCase):
    def output(self, rows, nc=2):
        return np.array(rows, dtype=np.float32).T[np.newaxis,:,:]

    def detect(self, out, **kw):
        args = dict(source=(640,480),target=(320,320),nc=2,confidence=.45,
                    nms_iou=.45)
        args.update(kw)
        return decode(out,**args)

    def test_centered_letterbox_inverts_padding(self):
        self.assertEqual(geometry((640,480),(320,320),'letterbox'),(.5,.5,0,40,0,40))
        out=self.output([[160,160,100,80,.9,.1]])
        self.assertEqual(self.detect(out)[0][2],(220,160,200,160))

    def test_top_left_letterbox_restores_landscape_coordinates(self):
        self.assertEqual(geometry((640,480),(320,320),'letterbox_top_left'),
                         (.5,.5,0,0,0,80))
        out=self.output([[160,120,100,80,.9,.1]])
        self.assertEqual(self.detect(out,mode='letterbox_top_left')[0][2],
                         (220,160,200,160))

    def test_top_left_letterbox_restores_portrait_coordinates(self):
        self.assertEqual(geometry((480,640),(320,320),'letterbox_top_left'),
                         (.5,.5,0,0,80,0))
        out=self.output([[120,160,80,100,.9,.1]])
        self.assertEqual(self.detect(out,source=(480,640),
                                    mode='letterbox_top_left')[0][2],
                         (160,220,160,200))

    def test_top_left_letterbox_truncates_resized_dimensions(self):
        self.assertEqual(geometry((641,480),(320,320),'letterbox_top_left'),
                         (320/641,239/480,0,0,0,81))
        # Centered mode retains its previous round-to-nearest behavior.
        self.assertEqual(geometry((641,480),(320,320),'letterbox'),
                         (320/641,.5,0,40,0,40))

    def test_top_left_letterbox_removes_boxes_inside_padding(self):
        bottom=self.output([[100,290,20,20,.9,.1]])
        right=self.output([[290,100,20,20,.9,.1]])
        self.assertEqual(self.detect(bottom,mode='letterbox_top_left'),[])
        self.assertEqual(self.detect(right,source=(480,640),
                                    mode='letterbox_top_left'),[])

    def test_24_class_output_preserves_highest_class_in_both_layouts(self):
        row=[160,120,100,80]+[0.0]*24
        row[4+23]=.9
        out=self.output([row],nc=24)
        result=self.detect(out,nc=24,mode='letterbox_top_left')
        self.assertEqual(result[0][0],23)
        self.assertEqual(result[0][2],(220,160,200,160))
        self.assertEqual(result,self.detect(out.transpose(0,2,1),nc=24,
                                           mode='letterbox_top_left'))
        with self.assertRaises(ValueError):
            self.detect(out,nc=11,mode='letterbox_top_left')

    def test_both_layouts(self):
        out=self.output([[160,160,100,80,.9,.1],[80,90,20,20,.1,.8]])
        self.assertEqual(self.detect(out),self.detect(out.transpose(0,2,1)))

    def test_class_aware_nms(self):
        out=self.output([[160,160,100,80,.9,.1],[161,160,100,80,.8,.1],
                         [160,160,100,80,.1,.7]])
        self.assertEqual([d[0] for d in self.detect(out)],[0,1])

    def test_invalid_and_padding_boxes_are_removed(self):
        out=self.output([[20,5,10,10,.9,.1],[160,160,0,20,.9,.1],
                         [float('nan'),160,20,20,.9,.1],[100,100,20,20,.1,.2]])
        self.assertEqual(self.detect(out),[])

    def test_clipping_and_stretch(self):
        out=self.output([[0,0,100,100,.9,.1]])
        self.assertEqual(self.detect(out,mode='stretch')[0][2],(0,0,100,75))

    def test_top_k_keeps_strongest_late_candidates(self):
        out=self.output([[30,100,10,10,.5,.1],[100,100,10,10,.7,.1],[200,100,10,10,.9,.1]])
        result=self.detect(out,max_candidates=2,max_detections=1)
        self.assertEqual(result[0][2][0],390)

    def test_reject_raw_heads_and_wrong_classes(self):
        for out in (np.zeros((1,3,10,10)),np.zeros((1,84,2100)),np.zeros((2,6,100))):
            with self.assertRaises(ValueError): self.detect(out)

    def test_labels_do_not_invent_names(self):
        self.assertEqual(class_name(5,()),'class_5')
        self.assertEqual(class_name(0,('triangle',)),'triangle')

    def frame(self,lab):
        return types.SimpleNamespace(width=lambda:320,height=lambda:240,
            get_statistics=lambda **kwargs: types.SimpleNamespace(l_mean=lambda:lab[0],
                a_mean=lambda:lab[1],b_mean=lambda:lab[2]))

    def test_shape_model_uses_lab_not_class_id_for_robot(self):
        result=tracking_candidates([(10,.9,(200,160,100,80))],self.frame((45,55,30)),(640,480),(1,2,3))
        self.assertEqual(result[0],(125,100,50,40,2000,1,(100,80,50,40)))

    def test_preview_only_colors_do_not_become_robot_targets(self):
        for lab in ((12,0,0),(80,-5,65)):
            result=tracking_candidates([(0,.9,(200,160,100,80))],self.frame(lab),(640,480),(1,2,3))
            self.assertEqual(result,[])

    def test_explicit_mapping_and_requested_color(self):
        det=[(0,.9,(200,160,100,80))]
        self.assertEqual(tracking_candidates(det,self.frame((0,0,0)),(640,480),(1,), (2,)),[])
        self.assertEqual(tracking_candidates(det,self.frame((0,0,0)),(640,480),(2,), (2,))[0][5],2)

if __name__=='__main__': unittest.main()
