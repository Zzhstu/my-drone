"""Geometric acceptance regressions, including first-view baseline false holes."""
import unittest
import numpy as np
import trimesh
from warehouse20_audit import has_translation_baseline,intersector,unobstructed


class WarehouseGeometry(unittest.TestCase):
    def test_baseline_can_span_both_sides_of_first_view(self):
        self.assertTrue(has_translation_baseline(np.array([[0,0,0],[.3,0,0],[-.3,0,0]])))
    def test_rotation_at_same_position_does_not_create_baseline(self):
        self.assertFalse(has_translation_baseline(np.array([[0,0,0],[0,0,0],[.1,0,0]])))
    def test_real_mesh_occludes_target(self):
        mesh=trimesh.creation.box([1,1,1]);ray=intersector(mesh.triangles)
        p=np.array([[0,0,1.],[2,0,1.]])
        visible=unobstructed(ray,p,np.array([[0,0,-1.],[0,0,-1.]]),np.array([0,0,-2.]))
        self.assertFalse(visible[0]);self.assertTrue(visible[1])


if __name__=='__main__':unittest.main()
