#!/usr/bin/env python3
"""Evidence gates for adaptive inspection and redundant-goal retirement."""
import sys
import unittest
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_coverage import EfficiencyPolicy,FrontierGoalMonitor,SurfaceEvidence
from exploration_grid import ExplorationGrid


class EfficiencyTest(unittest.TestCase):
    def update(self,p,now,stamp=None,cells=1000,goals=6,age=200,quality=.95,count=1000,cap=False):
        return p.update(now,now if stamp is None else stamp,cells,goals,age,
            dict(observed_surface_quality_ratio=quality,observed_surface_voxels=count,
                 surface_evidence_capacity_limited=cap),.2)

    def test_plateau_starts_inspection_without_three_recent_arrivals(self):
        p=EfficiencyPolicy()
        self.update(p,0)
        r=self.update(p,60)
        self.assertTrue(r['ready_to_inspect'])
        self.assertFalse(r['ready_to_return'])
        self.assertTrue(self.update(p,70)['ready_to_return'])

    def test_new_floor_discovery_prevents_quality_only_exit(self):
        p=EfficiencyPolicy();self.update(p,0)
        r=self.update(p,60,cells=1200,quality=1.)
        self.assertFalse(r['ready_to_inspect'])
        self.assertFalse(r['ready_to_return'])

    def test_plateau_scales_to_observed_area_but_keeps_absolute_floor(self):
        p=EfficiencyPolicy(area_fraction=.01)
        self.update(p,0,cells=15000)
        r=self.update(p,60,cells=15100)
        self.assertAlmostEqual(r['growth_limit_m2'],6.04)
        self.assertTrue(r['ready_to_inspect'])  # 4m² / ~600m², quality gate separate
        p.reset();self.update(p,0,cells=100)
        r=self.update(p,60,cells=150)
        self.assertAlmostEqual(r['growth_limit_m2'],2.)
        self.assertFalse(r['ready_to_inspect'])  # 2m² is not strictly below the limit

    def test_low_quality_small_map_or_capacity_cannot_pass(self):
        for kwargs in (dict(quality=.9499),dict(count=999),dict(cap=True)):
            p=EfficiencyPolicy();self.update(p,0,**kwargs)
            self.update(p,60,**kwargs)
            self.assertFalse(self.update(p,70,**kwargs)['ready_to_return'])

    def test_minimum_mission_and_total_observations_required(self):
        for kwargs in (dict(age=179),dict(goals=5)):
            p=EfficiencyPolicy();self.update(p,0,**kwargs)
            self.assertFalse(self.update(p,60,**kwargs)['ready_to_inspect'])

    def test_repeated_scan_and_quality_dip_do_not_complete_hold(self):
        p=EfficiencyPolicy();self.update(p,0);self.update(p,60)
        self.assertFalse(self.update(p,80,stamp=60)['ready_to_return'])
        self.assertFalse(self.update(p,81,quality=.9)['ready_to_return'])
        self.assertFalse(self.update(p,82)['ready_to_return'])
        self.assertTrue(self.update(p,92)['ready_to_return'])
        p.reset()
        self.assertFalse(self.update(p,93)['ready_to_inspect'])

    def test_invalid_thresholds_rejected(self):
        for kwargs in (dict(seconds=0),dict(quality=1.1),dict(area=float('nan'))):
            with self.assertRaises(ValueError):EfficiencyPolicy(**kwargs)

    def test_obsolete_frontier_needs_fresh_persistent_low_gain(self):
        m=FrontierGoalMonitor()
        self.assertFalse(m.obsolete((1,2),0,0,.1,4.))
        self.assertFalse(m.obsolete((1,2),4,0,.1,4.))
        self.assertTrue(m.obsolete((1,2),4,4,.1,4.))
        self.assertFalse(m.obsolete((2,3),5,5,.1,4.))

    def test_near_arrival_or_recovered_gain_prevents_retirement(self):
        m=FrontierGoalMonitor()
        m.obsolete((1,2),0,0,.1,4.)
        self.assertFalse(m.obsolete((1,2),4,4,.1,.5))
        self.assertFalse(m.obsolete((1,2),5,5,.1,4.))
        self.assertFalse(m.obsolete((1,2),9,9,2.,4.))
        self.assertFalse(m.obsolete((1,2),10,10,.1,4.))

    def test_inspection_waypoint_guarantees_original_view_baseline_with_arrival_error(self):
        g=ExplorationGrid([-7,-7,7,7]);g.known[:]=True;g.update_obstacles([])
        d,_=g.search(g.cell((-3.,0.)))
        s=SurfaceEvidence();surface=np.array([[.2,.2,1.4]])
        first=np.array([1.7,.2,1.4])
        for stamp in (1,2,3):s.observe(surface,first,stamp)
        mask,_=s.inspection_candidates(g,d,surface)
        xy=np.array([g.xy(tuple(c)) for c in np.argwhere(mask)])
        self.assertGreater(len(xy),0)
        self.assertTrue(np.all(np.linalg.norm(xy-first[:2],axis=1)>=s.baseline+.3-1e-9))
        old_bits=s.records[tuple(np.floor(surface[0]/s.voxel).astype(int))][1]
        for view in xy:
            for angle in np.linspace(0,2*np.pi,40):
                actual=view+.3*np.array([np.cos(angle),np.sin(angle)])
                theta=np.arctan2(actual[1]-surface[0,1],actual[0]-surface[0,0])
                bit=1<<int(((theta+np.pi)%(2*np.pi))/(2*np.pi)*8)
                self.assertFalse(old_bits&bit)

    def test_inspection_scores_count_deficient_voxels_not_just_representatives(self):
        g=ExplorationGrid([-10,-8,10,8]);g.known[:]=True;g.update_obstacles([])
        d,_=g.search(g.cell((0.,0.)))
        s=SurfaceEvidence()
        deficits=np.array([[-4.2,.2,1.],[-4.2,.2,1.4],[-4.2,.2,1.8],[4.2,.2,1.]])
        mask,gain=s.inspection_candidates(g,d,deficits)
        left=[gain[tuple(c)] for c in np.argwhere(mask) if g.xy(tuple(c))[0]<0]
        right=[gain[tuple(c)] for c in np.argwhere(mask) if g.xy(tuple(c))[0]>0]
        self.assertGreater(max(left),max(right))


if __name__=='__main__':
    unittest.main()
