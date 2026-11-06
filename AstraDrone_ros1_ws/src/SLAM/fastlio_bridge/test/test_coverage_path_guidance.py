#!/usr/bin/env python3
"""Planner checks with synthetic observed-safe cells, never scene truth."""
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from coverage_path_guidance import CoveragePathGuide
from exploration_grid import ExplorationGrid


class CoveragePathGuideTest(unittest.TestCase):
    def setUp(self):
        self.grid = ExplorationGrid([-3.,-3.,3.,3.],.15,.8,.4)
        self.grid.known[:] = True
        self.grid.safe[:] = True
        self.start = self.grid.cell((-2.7,-2.7))
        self.distances,_ = self.grid.search(self.start,backend='python')
        self.gain = np.zeros(self.grid.shape)

    def test_reachable_region_guides_frontier_without_overriding_safety(self):
        near = self.grid.cell((-2.4,-2.4))
        far = self.grid.cell((1.5,1.5))
        excluded = self.grid.cell((0.,0.))
        self.gain[near] = .6
        self.gain[far] = 18.
        self.gain[excluded] = 1000.
        self.grid.safe[excluded] = False
        options = np.asarray([near,far,excluded])
        selected,report = CoveragePathGuide().select(
            self.grid,options,self.gain,self.distances,clearance_weight=0.)
        self.assertEqual(selected,far)
        self.assertEqual(report['candidate_regions'],2)
        self.assertEqual(report['route_scope'],'observed-safe fixed-altitude regions')

    def test_world_region_key_survives_grid_growth(self):
        guide = CoveragePathGuide(region_size=2.)
        point = (1.1,-1.1)
        key = guide.region(point)
        self.assertIsNotNone(self.grid.grow_to_include(np.asarray([[4.,4.]]),margin=1.,chunk=2.))
        self.assertEqual(guide.region(self.grid.xy(self.grid.cell(point))),key)

    def test_unreachable_options_do_not_create_targets(self):
        target = self.grid.cell((1.,1.))
        distances = self.distances.copy()
        distances[target] = np.inf
        self.gain[target] = 10.
        selected,report = CoveragePathGuide().select(
            self.grid,np.asarray([target]),self.gain,distances)
        self.assertIsNone(selected)
        self.assertEqual(report['reason'],'no_safe_reachable_frontier')


if __name__ == '__main__':
    unittest.main()
