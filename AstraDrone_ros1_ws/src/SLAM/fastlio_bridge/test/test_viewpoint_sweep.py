import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from viewpoint_sweep import sweep_candidates,ScanDwell


class ViewpointSweepTest(unittest.TestCase):
    def test_only_safe_reachable_unvisited_space_is_requested(self):
        g=ExplorationGrid([0,0,12,12]);g.known[:]=True;g.update_obstacles([])
        d,_=g.search(g.cell((2,2)));visited=np.zeros(g.shape,bool);visited[:30]=True
        d[:,45:]=np.inf;focus=np.zeros(g.shape,bool);focus[20:50,10:50]=True
        before=g.safe.copy();mask,gain,report=sweep_candidates(g,d,visited,focus)
        self.assertTrue(mask.any());self.assertFalse((mask&visited).any())
        self.assertTrue(g.safe[mask].all());self.assertTrue(np.isfinite(d[mask]).all())
        self.assertTrue(focus[mask].all());self.assertTrue((gain[mask]>0).all())
        np.testing.assert_array_equal(before,g.safe)
        self.assertGreater(report['remaining_reachable_viewpoint_area_m2'],0)

    def test_obstacle_occlusion_prevents_cross_aisle_visitation_credit(self):
        from warehouse_explorer import Explorer
        e=Explorer.__new__(Explorer);e.grid=ExplorationGrid([0,0,12,12]);e.grid.known[:]=True
        wall=np.c_[np.full(60,6.),np.arange(.1,12,.2)]
        e.grid.update_obstacles(wall);e.inspected=np.zeros(e.grid.shape,bool)
        e.p=lambda k,v:4. if k=='inspection_radius' else v
        e.mark_inspected((4.5,6.))
        self.assertTrue(e.inspected[e.grid.cell((4.5,6.))])
        self.assertFalse(e.inspected[e.grid.cell((7.5,6.))])

    def test_tiny_quantization_pocket_does_not_block_refinement_forever(self):
        g=ExplorationGrid([0,0,12,12]);g.safe[:]=True;visited=np.ones(g.shape,bool)
        visited[5,5]=False;mask,_,_=sweep_candidates(g,np.ones(g.shape),visited)
        self.assertFalse(mask.any())

    def test_whole_remote_aisle_can_outweigh_small_near_pocket(self):
        from joint_view_policy import JointViewPolicy
        from flight_budget import FlightBudget
        g=ExplorationGrid([0,0,20,20]);g.known[:]=True;g.update_obstacles([])
        visited=np.ones(g.shape,bool)
        visited[15:20,15:25]=False
        visited[45:65,45:70]=False
        distances,_=g.search(g.cell((2.,2.)))
        mask,gain,report=sweep_candidates(g,distances,visited)
        near=g.cell((3.3,3.3));far=g.cell((9.3,9.3))
        self.assertGreater(gain[far],5*gain[near])
        self.assertLessEqual(gain[far],1.)
        cfg=dict(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True,
            joint_surface_start_seconds=0.,joint_surface_action_budget_seconds=180.,
            joint_surface_dwell_seconds=4.)
        policy=JointViewPolicy(lambda k,v:cfg.get(k,v));budget=FlightBudget()
        budget.observe_arm(False,1.);budget.observe_arm(True,2.)
        view=policy.select(g,(2.,2.),distances,distances,np.empty((0,2),int),np.empty(0),
            [],np.empty((0,3)),np.empty((0,3)),1.2,100.,budget,
            inspection_cells=np.array([near,far]),inspection_gains=np.array([gain[near],gain[far]])*16.,
            inspection_sources=['missing_sweep','missing_sweep'])
        self.assertIsNotNone(view);self.assertEqual(view.cell,far)

    def test_arrival_and_duplicate_scans_do_not_complete_dwell(self):
        dwell=ScanDwell(0,4,8)
        for t in np.arange(0,6,.5):
            complete,_=dwell.update(t,0,True);self.assertFalse(complete)
        self.assertEqual(dwell.scans,1)
        self.assertTrue(dwell.update(7,0,True)[1])

    def test_continuous_dwell_and_fresh_scans_complete(self):
        dwell=ScanDwell(0,4,8)
        for i in range(8):self.assertFalse(dwell.update(i*.5,i*.5,True)[0])
        self.assertTrue(dwell.update(4,4,True)[0])
        dwell=ScanDwell(0,4,8)
        for i in range(6):dwell.update(i*.5,i*.5,True)
        dwell.update(3,3,False)
        self.assertFalse(dwell.update(4,4,True)[0]);self.assertEqual(dwell.scans,1)

    def test_explorer_only_credits_region_after_real_dwell(self):
        from types import SimpleNamespace
        from warehouse_explorer import Explorer
        from joint_view_policy import View
        e=Explorer.__new__(Explorer);e.grid=ExplorationGrid([0,0,12,12])
        cell=e.grid.cell((4.1,4.1));v=View(cell,e.grid.xy(cell),'surface_inspection',dwell_s=4.)
        credited=[];events=[];e.joint_policy=SimpleNamespace(selected=v,
            complete_inspection=lambda view:credited.append(view),last_decision=0)
        e.joint_surface_observation=dict(view=v,dwell=ScanDwell(0,4,8))
        e.p=lambda k,d:d;e.publish=lambda *args:None
        e.joint_event=lambda *args,**kwargs:events.append(kwargs)
        e.goals_reached=0;e.visited=[];e.inspection_visited=[];e.failed_visits={};e.blacklist=[];e.target=cell
        for i in range(8):
            inputs={'scan':(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:i*.5))),0)}
            self.assertTrue(e.joint_finish_surface_observation(i*.5,inputs,v.xy))
            self.assertFalse(credited);self.assertIs(e.joint_policy.selected,v)
        inputs={'scan':(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:4.))),0)}
        self.assertFalse(e.joint_finish_surface_observation(4.,inputs,v.xy))
        self.assertEqual(credited,[v]);self.assertEqual(e.goals_reached,1)
        self.assertIsNone(e.joint_policy.selected)


if __name__=='__main__':unittest.main()
