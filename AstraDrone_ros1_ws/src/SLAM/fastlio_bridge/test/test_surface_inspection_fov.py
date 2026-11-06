"""Safe viewpoints still need to see the target in the measured sensor cone."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from exploration_coverage import SurfaceEvidence


class SurfaceInspectionFovTest(unittest.TestCase):
    def test_unreachable_dense_group_does_not_starve_an_observable_surface(self):
        grid=ExplorationGrid([0,0,14,14],.2,.8,.4)
        grid.known[:]=True;grid.update_obstacles([])
        d,_=grid.search(grid.cell((1.,1.)))
        # A tall dense patch outside the known room ranks first. The second
        # patch has a valid view and must survive a quota of one surface.
        inaccessible=np.array([[40.,40.,.2+i*.4] for i in range(10)])
        targets=np.vstack([inaccessible,[[7.,7.,1.2]]])
        evidence=SurfaceEvidence()
        mask,_=evidence.inspection_candidates(grid,d,targets,limit=1,range_m=5.,altitude=1.2,robust_quota=True)
        self.assertTrue(mask.any())
        self.assertEqual(evidence.last_inspection_report['contributing_groups'],1)
        self.assertEqual(evidence.last_inspection_report['groups_tried'],2)

    def test_tilted_frame_low_floor_needs_a_more_distant_view(self):
        grid=ExplorationGrid([0,0,18,18],.2,.8,.4)
        grid.known[:]=True;grid.update_obstacles([])
        d,_=grid.search(grid.cell((1.,1.)))
        target=np.array([[9.,9.,-.6]])
        evidence=SurfaceEvidence()
        near,_=evidence.inspection_candidates(grid,d,target,range_m=3.8,altitude=1.2)
        far,_=evidence.inspection_candidates(grid,d,target,range_m=5.,altitude=1.2)
        self.assertFalse(near.any());self.assertTrue(far.any())

    def test_floor_targets_are_not_scored_in_downward_blind_cone(self):
        grid=ExplorationGrid([0,0,14,14],.2,.8,.4)
        grid.known[:]=True;grid.update_obstacles([])
        d,_=grid.search(grid.cell((1.,1.)))
        target=np.array([[7.,7.,.2]])
        mask,_=SurfaceEvidence().inspection_candidates(grid,d,target,range_m=3.8,altitude=1.2)
        self.assertTrue(mask.any())
        for cell in np.argwhere(mask):
            horizontal=math.dist(grid.xy(cell),target[0,:2])
            angle=math.degrees(math.atan2(.2-.2-1.2,horizontal))
            self.assertGreaterEqual(angle,-22.2)

    def test_upper_targets_are_not_scored_above_sensor_cone(self):
        grid=ExplorationGrid([0,0,14,14],.2,.8,.4)
        grid.known[:]=True;grid.update_obstacles([])
        d,_=grid.search(grid.cell((1.,1.)))
        target=np.array([[7.,7.,5.]])
        mask,_=SurfaceEvidence().inspection_candidates(grid,d,target,range_m=3.8,altitude=1.2)
        self.assertFalse(mask.any())
