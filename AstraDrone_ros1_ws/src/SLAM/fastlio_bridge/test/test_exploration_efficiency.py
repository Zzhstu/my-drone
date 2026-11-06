#!/usr/bin/env python3
"""Inspection candidate baseline and deficit-weight regressions."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_coverage import SurfaceEvidence, ObservedMapAcceptance
from exploration_grid import ExplorationGrid


class ObservedMapAcceptanceTest(unittest.TestCase):
    def replay(self, gate, end=200, **overrides):
        result=False
        for second in range(end+1):
            values=dict(now=float(second),stamp=float(second),known_count=15000,resolution=.2,
                goals=6,mission_seconds=float(second),surface=dict(observed_surface_voxels=2000,
                observed_surface_quality_ratio=.90))
            values.update(overrides)
            result=gate.update(**values)
        return result

    def test_relaxed_acceptance_requires_maturity_and_dwell(self):
        g=ObservedMapAcceptance()
        self.assertFalse(self.replay(g,189))
        self.assertTrue(g.update(190,190,15000,.2,6,190,
            dict(observed_surface_voxels=2000,observed_surface_quality_ratio=.90)))

    def test_quality_small_map_goals_and_capacity_do_not_pass(self):
        for values in (dict(goals=5),dict(mission_seconds=179),dict(capacity_limited=True),
                dict(surface=dict(observed_surface_voxels=999,observed_surface_quality_ratio=1.)),
                dict(surface=dict(observed_surface_voxels=2000,observed_surface_quality_ratio=.899)),
                dict(surface=dict(observed_surface_voxels=2000,observed_surface_quality_ratio=1.,
                                  surface_evidence_capacity_limited=True))):
            self.assertFalse(self.replay(ObservedMapAcceptance(),**values))

    def test_duplicate_scans_do_not_advance_acceptance(self):
        g=ObservedMapAcceptance()
        self.assertFalse(self.replay(g,stamp=0.))
        self.assertFalse(g.report.get('window_ready',False))

    def test_active_discovery_does_not_pass(self):
        g=ObservedMapAcceptance()
        for second in range(220):
            accepted=g.update(second,second,10000+second*10,.2,6,second,
                dict(observed_surface_voxels=2000,observed_surface_quality_ratio=.95))
        self.assertFalse(accepted)
        self.assertGreater(g.report['new_known_area_m2'],g.report['area_threshold_m2'])

    def test_gap_and_explicit_reset_drop_old_evidence(self):
        for gap in (False,True):
            g=ObservedMapAcceptance();self.assertTrue(self.replay(g))
            if not gap:g.reset()
            self.assertFalse(g.update(204,204,15000,.2,6,204,
                dict(observed_surface_voxels=2000,observed_surface_quality_ratio=.95)))
            self.assertFalse(g.report['window_ready'])

    def test_quality_dip_restarts_ten_second_hold(self):
        g=ObservedMapAcceptance();self.assertFalse(self.replay(g,185))
        for second in range(186,196):
            quality=.89 if second==186 else .90
            self.assertFalse(g.update(second,second,15000,.2,6,second,
                dict(observed_surface_voxels=2000,observed_surface_quality_ratio=quality)))


class InspectionCandidateTest(unittest.TestCase):
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
