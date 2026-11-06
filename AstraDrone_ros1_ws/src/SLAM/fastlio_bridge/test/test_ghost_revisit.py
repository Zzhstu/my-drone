#!/usr/bin/env python3
"""Ghost-view selection regressions on observed grids, without ROS/Gazebo."""
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from ghost_revisit import (cleared_history_groups, history_group_priorities, observed_los,
                           occupied_voxel_keys, select_revisit_view,
                           suspect_groups, visible_target_count,
                           certify_3d_swept_route, diagnose_alternative_height_views,
                           validate_height_offsets)


class GhostRevisitTest(unittest.TestCase):
    def test_low_target_needs_real_sensor_height_and_longer_horizontal_view(self):
        self.assertEqual(visible_target_count((0.,0.,1.63),np.array([[2.5,0.,.44]]),set(),
            .15,-22.2,37.2),0)
        self.assertEqual(visible_target_count((0.,0.,1.63),np.array([[3.5,0.,.44]]),set(),
            .15,-22.2,37.2),1)

    def test_phase_time_does_not_charge_return_twice(self):
        grid=self.grid();target=np.array([[5.,5.,1.2]])
        grid.update_obstacles(target);distances,_=grid.search(grid.cell((2.,5.)))
        home=np.full(grid.shape,20.);diagnostics={}
        view,_=select_revisit_view(grid,distances,suspect_groups(target,1.2),(2.,5.),{},
            speed_mps=.5,remaining_seconds=100.,phase_remaining_seconds=15.,
            return_distances=home,diagnostics=diagnostics)
        self.assertIsNotNone(view)
        denied,_=select_revisit_view(grid,distances,suspect_groups(target,1.2),(2.,5.),{},
            speed_mps=.5,remaining_seconds=10.,phase_remaining_seconds=15.,
            return_distances=home,diagnostics=diagnostics)
        self.assertIsNone(denied)
        self.assertTrue(any(v.get('flight_return_budget') for v in diagnostics['groups'].values()))

    def grid(self):
        grid = ExplorationGrid([0,0,10,10],.2,.8,.4)
        grid.known[:] = True
        grid.update_obstacles([])
        return grid

    def test_reachable_observed_view_and_distinct_second_view(self):
        grid = self.grid()
        target = np.array([[5.,5.,1.2],[5.1,5.,1.2]])
        grid.update_obstacles(target)
        groups = suspect_groups(target,1.2)
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        first,key = select_revisit_view(grid,distances,groups,start,{})
        self.assertIsNotNone(first)
        self.assertTrue(grid.safe[first])
        self.assertTrue(observed_los(grid,grid.xy(first),groups[0][1]))
        second,key2 = select_revisit_view(grid,distances,groups,start,
                                          {key:[grid.xy(first)]})
        self.assertEqual(key,key2)
        self.assertGreaterEqual(np.linalg.norm(np.array(grid.xy(first))-
                                               np.array(grid.xy(second))),.7)
        exhausted,_ = select_revisit_view(grid,distances,groups,start,
                                          {key:[grid.xy(first),grid.xy(second)]})
        self.assertIsNone(exhausted)

    def test_wall_blocks_view_even_when_target_within_sensor_range(self):
        grid = self.grid()
        wall = np.array([[5.,y,1.2] for y in np.arange(0,10,.1)])
        target = np.array([[6.,5.,1.2]])
        grid.update_obstacles(np.vstack((wall,target)))
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        view,_ = select_revisit_view(grid,distances,
                                    suspect_groups(target,1.2),start,{})
        self.assertIsNone(view)

    def test_unknown_space_does_not_certify_line_of_sight(self):
        grid = self.grid()
        grid.known[:,24:26] = False
        grid.update_obstacles([])
        self.assertFalse(observed_los(grid,(3.,5.),(7.,5.)))

    def test_history_only_weights_actionable_current_suspects(self):
        history = {'records':[
            {'position':[5.,5.,1.2], 'state':'suspected',
             'actionable':True, 'priority':2.5},
            {'position':[5.1,5.,1.2], 'state':'cleared',
             'actionable':False, 'priority':100.},
            {'position':[7.,5.,1.2], 'state':'suspected',
             'actionable':False, 'priority':30.}]}
        weights = history_group_priorities(history,1.2)
        self.assertEqual(weights, {tuple(np.floor(np.array([5.,5.])/.8).astype(int)):2.5})

    def test_cleared_history_creates_separate_safe_view_targets(self):
        history = {'records':[
            {'position':[5.,5.,1.2], 'state':'cleared',
             'occupied_now':False, 'cleared_episodes':2},
            {'position':[5.1,5.,1.2], 'state':'cleared',
             'occupied_now':False, 'cleared_episodes':1},
            {'position':[7.,5.,1.2], 'state':'cleared',
             'occupied_now':True, 'cleared_episodes':4},
            {'position':[8.,5.,1.2], 'state':'suspected',
             'occupied_now':True, 'cleared_episodes':3}]}
        groups, weights = cleared_history_groups(history,1.2)
        self.assertEqual(len(groups),1)
        self.assertEqual(groups[0][2],2)
        self.assertEqual(len(weights),1)
        self.assertGreater(weights[groups[0][0]],0)
        grid = self.grid()
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        view,key = select_revisit_view(grid,distances,groups,start,{},
                                      priorities=weights,max_attempts=1)
        self.assertIsNotNone(view)
        self.assertTrue(grid.safe[view])
        self.assertTrue(observed_los(grid,grid.xy(view),groups[0][1]))
        exhausted,_ = select_revisit_view(grid,distances,groups,start,
                                          {key:[grid.xy(view)]},priorities=weights,
                                          max_attempts=1)
        self.assertIsNone(exhausted)

    def test_low_voxel_is_a_view_target_without_changing_flight_grid(self):
        low = np.array([[5.,5.,.25]])
        self.assertEqual(suspect_groups(low,1.2),[])
        groups = suspect_groups(low,1.2,height_below=1.25,height_above=.85)
        self.assertEqual(len(groups),1)
        grid = self.grid()
        grid.update_obstacles(low)
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        view,_ = select_revisit_view(grid,distances,groups,start,{})
        self.assertIsNotNone(view)
        self.assertTrue(grid.safe[view])

    def test_time_budget_and_evidence_weight_change_choice(self):
        grid = self.grid()
        target = np.array([[4.,5.,1.2],[7.,5.,1.2]])
        grid.update_obstacles(target)
        groups = suspect_groups(target,1.2)
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        home_distances,_ = grid.search(grid.cell(start))
        near = tuple(np.floor(np.array([4.,5.])/.8).astype(int))
        far = tuple(np.floor(np.array([7.,5.])/.8).astype(int))
        weighted = {near:1.,far:50.}
        self.assertEqual(suspect_groups(target,1.2,limit=1,
                                        priorities=weighted)[0][0],far)
        _,chosen = select_revisit_view(grid,distances,groups,start,{},
            priorities=weighted,return_distances=home_distances,
            remaining_seconds=100.,speed_mps=.5,dwell_seconds=2.)
        self.assertEqual(chosen,far)
        _,chosen = select_revisit_view(grid,distances,groups,start,{},
            priorities=weighted,return_distances=home_distances,
            remaining_seconds=7.,speed_mps=.5,dwell_seconds=2.)
        self.assertEqual(chosen,near)

    def test_time_budget_reserves_return_leg(self):
        grid = self.grid()
        groups = suspect_groups([[5.,5.,1.2]],1.2)
        start = (2.,5.)
        distances,_ = grid.search(grid.cell(start))
        view,_ = select_revisit_view(grid,distances,groups,start,{},
                                     speed_mps=.5,dwell_seconds=2.,
                                     remaining_seconds=4.,
                                     return_distances=distances)
        self.assertIsNone(view)

    def test_3d_proxy_rejects_mapped_blocker_and_out_of_fov_target(self):
        view=(2.,5.,1.2)
        low=(5.,5.,.25)
        self.assertEqual(visible_target_count(view,[low],set()),1)
        blocker=occupied_voxel_keys([[3.5,5.,.725]],.15)
        self.assertEqual(visible_target_count(view,[low],blocker),0)
        self.assertEqual(visible_target_count(view,[[5.,5.,-1.]],set()),0)

    def test_3d_proxy_preserves_safe_grid_and_attempt_limit(self):
        grid=self.grid()
        target=np.array([[5.,5.,.25]])
        groups=suspect_groups(target,1.2,height_below=1.25)
        distances,_=grid.search(grid.cell((2.,5.)))
        view,key=select_revisit_view(grid,distances,groups,(2.,5.),{},
            target_points={groups[0][0]:target},occupied_voxels=set(),
            sensor_altitude=1.2)
        self.assertIsNotNone(view)
        self.assertTrue(grid.safe[view])
        blocked,_=select_revisit_view(grid,distances,groups,(2.,5.),{},
            target_points={groups[0][0]:target},
            occupied_voxels=occupied_voxel_keys([[5.,5.,.25]],.15),
            sensor_altitude=3.)
        self.assertIsNone(blocked)
        exhausted,_=select_revisit_view(grid,distances,groups,(2.,5.),
            {key:[grid.xy(view)]},max_attempts={key:1},
            target_points={key:target},occupied_voxels=set(),
            sensor_altitude=1.2)
        self.assertIsNone(exhausted)

    def test_3d_vertical_sweep_needs_every_voxel_explicitly_free(self):
        route=[(.5,.5,.5),(.5,.5,2.5)]
        free={(0,0,z) for z in range(3)}
        def check(known, occupied=(), radius=.1, half_height=.1,
                  free_stamp=10., occupied_stamp=10.):
            return certify_3d_swept_route(
                route,known,occupied,1.,radius,half_height,
                free_stamp,occupied_stamp,10.,1.)
        self.assertTrue(check(free)['certified'])
        self.assertEqual(check(free-{(0,0,1)})['reason'],'unknown')
        self.assertEqual(check(free,{(0,0,1)})['reason'],'occupied')
        self.assertEqual(check(free,free_stamp=8.)['reason'],'stale_free_evidence')
        self.assertEqual(check(free,occupied_stamp=8.)['reason'],'stale_occupied_evidence')

    def test_3d_sweep_inflates_horizontal_and_vertical_body(self):
        route=[(.5,.5,.5),(.5,.5,2.5)]
        center={(0,0,z) for z in range(3)}
        def check(radius, half_height, known):
            return certify_3d_swept_route(
                route,known,set(),1.,radius,half_height,10.,10.,10.,1.)
        self.assertTrue(check(.1,.1,center)['certified'])
        horizontal=check(.6,.1,center)
        self.assertEqual(horizontal['reason'],'unknown')
        self.assertNotEqual(horizontal['first_blocked_voxel'][:2],(0,0))
        vertical=check(.1,.6,center)
        self.assertEqual(vertical['reason'],'unknown')
        self.assertNotIn(vertical['first_blocked_voxel'][2],(0,1,2))

    def test_3d_sweep_checks_interior_of_horizontal_route(self):
        route=[(.5,.5,.5),(2.5,.5,.5)]
        endpoints={(0,0,0),(2,0,0)}
        result=certify_3d_swept_route(route,endpoints,set(),1.,.1,.1,
                                      10.,10.,10.,1.)
        self.assertEqual(result['reason'],'unknown')
        self.assertEqual(result['first_blocked_voxel'],(1,0,0))

    def test_3d_sweep_treats_boundary_contact_and_large_routes_conservatively(self):
        contact=certify_3d_swept_route(
            [(.5,.5,.5),(.5,.5,1.5)],{(0,0,0),(0,0,1)},set(),
            1.,.5,.1,10.,10.,10.,1.)
        self.assertEqual(contact['reason'],'unknown')
        self.assertNotEqual(contact['first_blocked_voxel'][:2],(0,0))
        long_route=certify_3d_swept_route(
            [(0.,0.,0.),(100.,0.,0.)],set(),set(),
            1.,.1,.1,10.,10.,10.,1.,max_checked=10)
        self.assertEqual(long_route['reason'],'resource_limit')

    def test_altitude_diagnostic_reports_geometry_without_flight_authority(self):
        grid=self.grid()
        target=np.array([[5.,5.,.25]])
        grid.update_obstacles(target)
        groups=suspect_groups(target,1.2,height_below=1.25)
        start=(2.,5.)
        distances,_=grid.search(grid.cell(start))
        current,key=select_revisit_view(
            grid,distances,groups,start,{},maximum_range=1.6,
            target_points={groups[0][0]:target},occupied_voxels=set(),
            sensor_altitude=1.2)
        self.assertIsNone(current)
        views=diagnose_alternative_height_views(
            grid,distances,groups,start,{groups[0][0]:target},set(),
            set(),1.2,[-.55],10.,10.,10.,1.,
            minimum_range=1.2,maximum_range=1.6)
        self.assertEqual(len(views),1)
        self.assertGreater(views[0]['proxy_target_gain'],0)
        self.assertFalse(views[0]['flight_authorized'])
        self.assertEqual(views[0]['vertical_sweep']['reason'],'unknown')

    def test_invalid_vertical_offsets_and_route_rejected(self):
        for offsets in ([],[0.],[-.4,-.4],[float('nan')],[2.1]):
            with self.assertRaises(ValueError):
                validate_height_offsets(offsets)
        with self.assertRaises(ValueError):
            certify_3d_swept_route([(0.,0.,0.)],set(),set(),.15,.45,.55,
                                    1.,1.,1.,1.)


if __name__ == '__main__':
    unittest.main()
