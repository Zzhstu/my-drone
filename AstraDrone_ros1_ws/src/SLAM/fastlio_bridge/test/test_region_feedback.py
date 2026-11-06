#!/usr/bin/env python3
"""Observed-task memory and recorded late-cost decision regressions."""
import json,sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_region_memory import BenefitClaim,RegionMemory,surface_claim,MeasuredSurfaceSupport,floor_deficit_components
from exploration_grid import ExplorationGrid
from exploration_coverage import SurfaceEvidence,visible_unknown_gain
from joint_view_policy import JointViewPolicy
from flight_budget import FlightBudget


class RegionFeedbackTest(unittest.TestCase):
    def space(self,support=frozenset()):return BenefitClaim(('space',0,0,-1,-1),(1.,1.,0.),2.,support=support)
    def fail(self,m,c,now=100.,valid=True):return m.finish((c,),{}, {},now,(1.,2.),valid)

    def test_two_observations_cool_and_timer_does_not_renew_reward(self):
        m=RegionMemory();c=self.space();self.fail(m,c);self.fail(m,c,110.)
        self.assertEqual(m.assess((c,),111.,(1.,2.))[0],0.)
        self.assertEqual(m.assess((c,),500.,(1.,2.))[0],0.)

    def test_invalid_stale_or_incomplete_observation_does_not_cool(self):
        m=RegionMemory();c=self.space()
        for _ in range(5):self.fail(m,c,valid=False)
        self.assertEqual(m.records[c.key]['attempts'],0)
        self.assertEqual(m.assess((c,),500.,(1.,2.))[0],1.)

    def test_new_unknown_cells_reopen_only_affected_request(self):
        m=RegionMemory();old=self.space();self.fail(m,old);self.fail(m,old)
        fresh=self.space(frozenset((i,0) for i in range(20)))
        self.assertEqual(m.assess((fresh,),120.,(1.,2.))[0],1.)
        self.assertEqual(m.assess((fresh,),121.,(1.,2.))[0],1.)

    def test_opposite_face_and_height_are_independent(self):
        m=RegionMemory();a=surface_claim('boundary',(1.,1.,1.),(1.,0.));b=surface_claim('boundary',(1.,1.,1.),(-1.,0.));c=surface_claim('boundary',(1.,1.,.1),(1.,0.))
        self.fail(m,a);self.fail(m,a)
        self.assertEqual(m.assess((b,),120.,(2.,1.))[0],1.)
        self.assertEqual(m.assess((c,),120.,(2.,1.))[0],1.)
        self.assertNotEqual(a.key,b.key);self.assertNotEqual(a.key,c.key)

    def test_surface_low_yield_shared_with_ordinary_frontier_not_other_face(self):
        m=RegionMemory();a=surface_claim('boundary',(1.,1.,1.),(1.,0.));self.fail(m,a)
        self.assertLess(m.assess((self.space(),),101.,(1.,2.))[0],1.)
        self.assertEqual(m.assess((surface_claim('boundary',(1.,1.,1.),(-1.,0.)),),101.,(2.,1.))[0],1.)

    def test_already_seen_but_newly_qualified_keys_are_useful(self):
        m=RegionMemory();c=self.space();after={c.key:dict(qualified=frozenset((i,0,0) for i in range(9)))}
        result=m.finish((c,),{},after,100.,(1.,2.));self.assertTrue(result[0]['useful']);self.assertEqual(result[0]['surface'],0)

    def test_critical_new_view_is_bounded_after_three_unproductive_attempts(self):
        m=RegionMemory();c=surface_claim('boundary',(1.,1.,1.),(1.,0.))
        for xy in ((2.,1.),(1.,2.),(0.,1.)):m.finish((c,),{},{},100.,xy)
        self.assertEqual(m.assess((c,),200.,(1.,0.))[0],0.)

    def test_frontier_claims_belong_to_unknown_targets_not_viewpoint(self):
        g=ExplorationGrid([0,0,12,12]);g.known[:]=True;g.known[:,40:]=False;g.update_obstacles(np.empty((0,3)))
        area,claims=visible_unknown_gain(g,[g.cell((6.,6.))],return_claims=True)
        self.assertGreater(area[0],0.);self.assertTrue(claims[0]);self.assertTrue(all(c.target[0]>=8. for c in claims[0]))
        self.assertEqual(sum(c.mass for c in claims[0]),area[0])

    def test_measured_face_holes_do_not_get_missing_face_protection(self):
        points=np.asarray([[1.,y,z] for y in np.arange(.2,2.2,.1) for z in np.arange(.3,1.8,.1)])
        index=MeasuredSurfaceSupport(points,[0.,0.,0.])
        seen,count=index.assess('boundary',(1.1,1.2,.1),(1.,0.))
        self.assertTrue(seen);self.assertGreaterEqual(count,8)
        opposite,index_count=index.assess('boundary',(1.7,1.2,.1),(-1.,0.))
        self.assertFalse(opposite);self.assertEqual(index_count,0)

    def test_fine_density_growth_is_not_new_target_discovery(self):
        m=RegionMemory();c=surface_claim('boundary',(1.,1.,1.),(1.,0.))
        before={c.key:dict(target=frozenset([(1,1,1)]))}
        after={c.key:dict(target=frozenset([(1,1,1),(1,1,2)]))}
        rows=m.finish((c,),before,after,100.,(2.,1.));self.assertFalse(rows[0]['useful'])
        discovered=m.finish((c,),{},after,101.,(2.,1.));self.assertTrue(discovered[0]['useful'])

    def test_height_keys_follow_ground_not_global_tilt(self):
        a=surface_claim('boundary',(1.,1.,.11),(1.,0.),height_origin=.01)
        b=surface_claim('boundary',(1.,1.,-.11),(1.,0.),height_origin=-.21)
        self.assertEqual(a.key,b.key)

    def test_short_measured_shelf_end_is_not_permanently_missing(self):
        points=np.asarray([[1.,y,z] for y in np.arange(.2,.81,.05) for z in np.arange(.3,1.81,.1)])
        support=MeasuredSurfaceSupport(points,[0.,0.,0.])
        self.assertTrue(support.assess('boundary',(1.1,.5,.1),(1.,0.))[0])
        self.assertFalse(support.assess('boundary',(1.7,.5,.1),(-1.,0.))[0])

    def test_observed_noncritical_face_capture_still_excludes_opposite_plane(self):
        g=ExplorationGrid([0,0,6,6]);g.known[:]=True;g.update_obstacles(np.empty((0,3)))
        s=SurfaceEvidence();s.current_keys=np.asarray([[2,2,2],[4,2,2]])
        s.current_qualified_keys=np.empty((0,3));s.current_points=np.array([[1.,1.,1.],[1.8,1.,1.]])
        c=surface_claim('boundary',(1.,1.,1.),(1.,0.),critical=False)
        snap=RegionMemory().capture((c,),g,s)
        self.assertEqual(snap[c.key]['surface'],frozenset([(2,2,2)]))

    def test_new_critical_success_does_not_restart_all_old_refinement(self):
        values=dict(joint_region_feedback_enabled=True,joint_stop_low_gain_surface=True)
        p=JointViewPolicy(lambda k,v:values.get(k,v))
        p.observe_surface_gain(0);p.observe_surface_gain(0);self.assertTrue(p.surface_stopped)
        p.observe_surface_gain(100);self.assertTrue(p.surface_stopped)
        p=JointViewPolicy(lambda k,v:dict(joint_stop_low_gain_surface=True).get(k,v))
        p.observe_surface_gain(0);p.observe_surface_gain(0);p.observe_surface_gain(100);self.assertFalse(p.surface_stopped)

    def test_floor_refinement_targets_stay_under_measured_roof(self):
        g=ExplorationGrid([0,0,20,20]);g.known[:]=True;g.update_obstacles(np.empty((0,3)))
        xy=np.asarray([(x,y) for x in np.arange(.1,8.,.2) for y in np.arange(.1,8.,.2)])
        floor=xy[(np.linalg.norm(xy-np.array([4.,4.]),axis=1)>1.0)]
        points=np.vstack((np.c_[floor,np.zeros(len(floor))],np.c_[xy,np.full(len(xy),2.5)]))
        s=SurfaceEvidence();s.current_points=points;distances,_=g.search(g.cell((4.,4.)))
        known=g.known.copy();safe=g.safe.copy()
        s.missing_geometry_candidates(g,distances,points,1.2,task_claims_enabled=True)
        claims=[c for rows in s.last_missing_geometry_claims['floor'].values() for c in rows]
        self.assertTrue(claims);self.assertTrue(all(c.target[0]<8.8 and c.target[1]<8.8 for c in claims))
        self.assertGreater(s.last_missing_geometry_report['deferred_floor_outside_observed_roof'],0)
        np.testing.assert_array_equal(g.known,known);np.testing.assert_array_equal(g.safe,safe)

    def test_capture_uses_static_map_and_never_edits_safety(self):
        m=RegionMemory();g=ExplorationGrid([0,0,6,6]);g.known[:]=True;g.update_obstacles(np.empty((0,3)));s=SurfaceEvidence();s.current_keys=np.array([[2,2,2]]);s.current_qualified_keys=np.empty((0,3));s.current_points=np.array([[1.,1.,1.]])
        before=g.safe.copy();snapshot=m.capture((self.space(),),g,s)
        self.assertEqual(len(snapshot[self.space().key]['surface']),1);np.testing.assert_array_equal(g.safe,before)

    def select_cost(self,go,home,direct,area,now=400.,safe=True,old=False,returning=False):
        g=ExplorationGrid([0,0,20,20]);g.known[:]=True;g.update_obstacles(np.empty((0,3)))
        start=g.cell((2.1,2.1));target=g.cell((10.1,2.1));d=np.full(g.shape,np.inf);h=d.copy();d[start]=0.;d[target]=go/2.6;h[start]=direct/2.6;h[target]=home/2.6;g.safe[target]=safe
        p=JointViewPolicy(lambda k,v:dict(joint_region_feedback_enabled=True,joint_lookahead_depth=1).get(k,v));p.surface_stopped=True
        b=FlightBudget(speed=.5);b.observe_arm(False,32.);b.observe_arm(True,33.)
        args=(g,(2.1,2.1),d,h,np.array([target]),np.array([area]),[],np.empty((0,3)),np.empty((0,3)),1.2,now,b)
        claim=self.space()
        with patch('joint_view_policy.visible_unknown_gain',return_value=(np.array([area]),[(claim,)])):
            if old:
                p.p['joint_tail_min_area_per_extra_s']=0.;p.select(*args);p.p['joint_tail_min_area_per_extra_s']=.1
            v=p.select(*args,return_phase=returning)
        return v,p

    def test_recorded_370s_useful_on_return_target_survives(self):
        v,p=self.select_cost(44.2739105,26.5569553,55.6616479,2.8,370.127)
        self.assertIsNotNone(v);self.assertAlmostEqual(v.extra_return_seconds,16.1692179,places=5)

    def test_recorded_402s_long_detour_is_rejected_including_old_target(self):
        for old in (False,True):
            v,p=self.select_cost(54.6031703,66.2062987,26.0369553,2.16,401.905,old=old)
            self.assertIsNone(v);self.assertGreater(p.report['rejected'].get('tail_low_incremental_value',0),0)

    def test_safety_return_and_budget_dominate_feedback(self):
        self.assertIsNone(self.select_cost(1.,1.,1.,20.,safe=False)[0])
        self.assertIsNone(self.select_cost(1.,1.,1.,20.,returning=True)[0])
        self.assertIsNone(self.select_cost(100.,100.,1.,20.,now=570.)[0])

    def test_unobserved_opposite_face_survives_global_refinement_stop(self):
        g=ExplorationGrid([0,0,12,12]);g.known[:]=True;g.update_obstacles(np.empty((0,3)))
        position=(2.1,2.1);d,_=g.search(g.cell(position));b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        values=dict(joint_region_feedback_enabled=True,joint_surface_inspection_enabled=True,
            joint_unseen_geometry_enabled=True,joint_surface_action_budget_seconds=240.,joint_lookahead_depth=1)
        p=JointViewPolicy(lambda k,v:values.get(k,v));p.surface_stopped=True
        front=surface_claim('boundary',(3.,3.,1.),(1.,0.));back=surface_claim('boundary',(3.,3.,1.),(-1.,0.))
        self.fail(p.region_memory,front);self.fail(p.region_memory,front)
        cell=g.cell((3.1,2.1))
        v=p.select(g,position,d,d,np.empty((0,2),int),np.empty(0),[],np.empty((0,3)),np.empty((0,3)),1.2,200.,b,
            inspection_cells=np.array([cell]),inspection_gains=np.array([16.]),inspection_sources=['missing_boundary'],
            inspection_regions=[(back.key,)],inspection_claims=[(back,)])
        self.assertIsNotNone(v);self.assertIn(back,v.benefit_claims);self.assertEqual(v.feedback_scale,1.)

    def test_observed_face_small_hole_cannot_bypass_refinement_stop(self):
        g=ExplorationGrid([0,0,12,12]);g.known[:]=True;g.update_obstacles(np.empty((0,3)))
        position=(2.1,2.1);d,_=g.search(g.cell(position));b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        values=dict(joint_region_feedback_enabled=True,joint_surface_inspection_enabled=True,
            joint_unseen_geometry_enabled=True,joint_surface_action_budget_seconds=240.,joint_lookahead_depth=1)
        p=JointViewPolicy(lambda k,v:values.get(k,v));p.surface_stopped=True
        c=surface_claim('boundary',(3.,3.,.1),(1.,0.),critical=False,support_voxels=20)
        v=p.select(g,position,d,d,np.empty((0,2),int),np.empty(0),[],np.empty((0,3)),np.empty((0,3)),1.2,200.,b,
            inspection_cells=np.array([g.cell((3.1,2.1))]),inspection_gains=np.array([16.]),
            inspection_sources=['missing_boundary'],inspection_regions=[(c.key,)],inspection_claims=[(c,)])
        self.assertIsNone(v);self.assertGreater(p.report['rejected'].get('observed_geometry_refinement_stopped',0),0)

    def test_floor_patch_is_distinguished_from_long_thin_border(self):
        mask=np.zeros((20,20),bool);mask[2:18,2]=True;mask[10:15,10:15]=True
        labels,parts=floor_deficit_components(mask,.2)
        fringe=parts[int(labels[2,2])];patch=parts[int(labels[12,12])]
        self.assertGreater(fringe['area_m2'],.5);self.assertEqual(fringe['core_cells'],0)
        self.assertGreater(patch['area_m2'],.5);self.assertGreater(patch['core_cells'],0)
        self.assertEqual(labels[0,0],0)

    def test_real_scene_transition_reopens_but_repeated_receipt_does_not(self):
        m=RegionMemory();c=self.space();self.fail(m,c);self.fail(m,c)
        row=dict(position=[1.,1.,1.],state='suspected',suspected_episodes=1,cleared_episodes=0,occupied_now=True)
        m.notify_scene([row],101.);self.assertEqual(m.assess((c,),102.,(1.,2.))[0],0.)
        row.update(state='cleared',cleared_episodes=1,occupied_now=False)
        m.notify_scene([row],103.);self.assertGreater(m.assess((c,),104.,(1.,2.))[0],0.)
        self.fail(m,c,110.);self.fail(m,c,111.);m.notify_scene([row],112.)
        self.assertEqual(m.assess((c,),113.,(1.,2.))[0],0.)

    def test_home_zero_ghost_state_cannot_indirectly_reopen_task(self):
        m=RegionMemory();c=self.space();self.fail(m,c);self.fail(m,c)
        row=dict(position=[1.,1.,1.],state='suspected')
        m.notify_scene([row],100.,(1.,1.),5.);row['state']='cleared';m.notify_scene([row],101.,(1.,1.),5.)
        self.assertEqual(m.assess((c,),102.,(1.,2.))[0],0.)

    def test_new_measured_face_support_can_reopen_exhausted_request(self):
        m=RegionMemory();c=surface_claim('boundary',(1.,1.,1.),(1.,0.))
        for xy in ((2.,1.),(1.,2.),(0.,1.)):m.finish((c,),{},{},100.,xy)
        fresh=surface_claim('boundary',(1.,1.,1.),(1.,0.),support_voxels=8)
        self.assertGreater(m.assess((fresh,),102.,(2.,1.))[0],0.)

    def test_task_export_keeps_unresolved_state_without_raw_sets(self):
        m=RegionMemory();c=self.space();self.fail(m,c);self.fail(m,c)
        output=json.loads(json.dumps(m.export(110.)))
        self.assertEqual(output['requests'][0]['state'],'low_yield_unresolved')
        self.assertEqual(output['requests'][0]['low_yield_count'],2)
        self.assertNotIn('signature',output['requests'][0])

    def test_graph_change_clears_spatial_memory_not_action_time(self):
        p=JointViewPolicy();self.fail(p.region_memory,self.space());p.used_surface_seconds=19.;p.invalidate_coordinates()
        self.assertFalse(p.region_memory.records);self.assertEqual(p.used_surface_seconds,19.)

    def test_capacity_never_claims_completion(self):
        m=RegionMemory(capacity=1);m.entry(self.space());c=surface_claim('boundary',(8.,8.,1.),(1.,0.));scale,state=m.weight(c,100.,(1.,2.))
        self.assertEqual(scale,1.);self.assertEqual(state,'capacity_unresolved');self.assertTrue(m.capacity_limited)

    def test_floor_claim_summary_is_json_serializable(self):json.dumps(surface_claim('floor',(1.,2.,0.)).summary())


if __name__=='__main__':unittest.main()
