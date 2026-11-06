import tempfile
import unittest
from pathlib import Path
import numpy as np
from PIL import Image
import torch
from losses import photometric,edge_loss
from dataset import Masks,load


class MaskSemantics(unittest.TestCase):
    def test_edges_exclude_mask_crossings(self):
        pred=torch.rand(20,20,3,requires_grad=True);target=torch.rand_like(pred)
        mask=torch.ones(20,20,dtype=torch.bool);mask[5:15,5:15]=False
        changed=target.clone();changed[~mask]=100
        a=edge_loss(pred,target,mask);b=edge_loss(pred,changed,mask)
        gradient=torch.autograd.grad(a,pred)[0]
        self.assertEqual(float(a),float(b));self.assertEqual(float(gradient[~mask].abs().max()),0.)
        empty=edge_loss(pred,target,torch.zeros_like(mask));self.assertEqual(float(empty),0.)

    def test_ignored_rgb_cannot_change_loss_or_gradient(self):
        torch.manual_seed(4);prediction=torch.rand(40,40,3,requires_grad=True);target=torch.rand_like(prediction)
        mask=torch.ones(40,40,dtype=torch.bool);mask[14:26,14:26]=False
        altered=target.detach().clone();altered[~mask]=100
        a,b=photometric(prediction,target,mask);loss=.8*a+.2*b
        gradient=torch.autograd.grad(loss,prediction)[0]
        c,d=photometric(prediction,altered,mask);changed=.8*c+.2*d
        changed_gradient=torch.autograd.grad(changed,prediction)[0]
        self.assertEqual(float(loss),float(changed));self.assertTrue(torch.equal(gradient,changed_gradient))
        self.assertEqual(float(gradient[~mask].abs().max()),0.)

    def test_white_is_ordinary_l1_and_black_has_no_nan_fallback(self):
        pred=torch.zeros(20,20,3,requires_grad=True);target=torch.ones_like(pred)
        l1,_=photometric(pred,target,torch.ones(20,20,dtype=torch.bool));self.assertEqual(float(l1),1.)
        with self.assertRaises(ValueError):photometric(pred,target,torch.zeros(20,20,dtype=torch.bool))

    @unittest.skipUnless(torch.cuda.is_available(),'CUDA required')
    def test_cuda_pooling_layout_cannot_leak_ignored_pixels(self):
        pred=torch.rand(96,128,3,device='cuda',requires_grad=True);target=torch.rand_like(pred)
        mask=torch.ones(96,128,dtype=torch.bool,device='cuda');mask[30:65,40:90]=False
        a,b=photometric(pred,target,mask);gradient=torch.autograd.grad(.8*a+.2*b,pred)[0]
        self.assertEqual(float(gradient[~mask].abs().max()),0.)

    def test_resize_and_boundary_exclusion_are_conservative(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'mask.png';a=np.full((40,40),255,np.uint8);a[16:24,16:24]=0;Image.fromarray(a).save(path)
            plain=Masks([path],20,20,0)[0];expanded=Masks([path],20,20,2)[0]
            self.assertTrue((expanded<=plain).all());self.assertLess(int(expanded.sum()),int(plain.sum()))

    def test_strict_photo_gate_remains_available(self):
        root=Path(__file__).resolve().parents[2]
        with self.assertRaisesRegex(ValueError,'Strict photo completeness'):
            load(root/'lio_dataset_runs/warehouse20_20261102_capture',96,require_complete_photos=True)


if __name__=='__main__':unittest.main()
