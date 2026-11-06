#!/usr/bin/env python3
"""Unseen floor / back-boundary sensing requests; never synthetic map fills."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from exploration_coverage import SurfaceEvidence


def floor_points(hole=None,tilt=(0.,0.)):
    x,y=np.meshgrid(np.arange(.1,12.,.2),np.arange(.1,12.,.2))
    p=np.c_[x.ravel(),y.ravel(),(tilt[0]*x+tilt[1]*y).ravel()]
    if hole is not None:
        x0,x1,y0,y1=hole
        p=p[~((p[:,0]>x0)&(p[:,0]<x1)&(p[:,1]>y0)&(p[:,1]<y1))]
    return p


class MissingGeometryTest(unittest.TestCase):
    def test_height_field_is_used_for_actual_sensor_visibility(self):
        g,d=self.grid();p=floor_points((7.,10.,7.,10.),tilt=(.06,-.04));s=SurfaceEvidence()
        heights=lambda xy:1.2+np.asarray(xy)[...,0]*.06-np.asarray(xy)[...,1]*.04
        captured=[]
        def ray(view,targets,*args,**kwargs):
            captured.append(np.array(view));return 1
        with patch('ghost_revisit.visible_target_count',side_effect=ray):
            s.missing_geometry_candidates(g,d,p,1.2,view_altitude=heights)
        self.assertTrue(captured)
        up=np.array([-.06,.04,1.]);up/=np.linalg.norm(up)
        for sensor in captured:
            body=sensor-.13*up
            self.assertAlmostEqual(body[2],float(heights(body[:2])))

    def test_narrow_safe_lattice_view_is_not_skipped_by_polar_sampling(self):
        g,_=self.grid();p=floor_points((7.,10.,7.,10.));s=SurfaceEvidence()
        # Only this narrow observed pocket is reachable. It lies between
        # the old 16-angle/four-radius samples but can really see the hole.
        g.safe[:]=False;cell=g.cell((4.1,4.1));g.safe[cell]=True
        distances=np.full(g.shape,np.inf);distances[cell]=4.
        before=g.safe.copy()
        mask,gain=s.missing_geometry_candidates(g,distances,p,1.2)
        self.assertTrue(mask[cell]);self.assertGreater(gain[cell],0.)
        self.assertEqual(mask.sum(),1)
        self.assertGreater(s.last_missing_geometry_report['visible_floor_targets'],0)
        np.testing.assert_array_equal(before,g.safe)

    def test_indoor_focus_requires_measured_ceiling(self):
        ground=floor_points();plane,on=SurfaceEvidence.measured_ceiling(ground,1.2)
        self.assertIsNone(plane);self.assertFalse(on.any())
        roof=ground.copy();roof[:,2]=2.8
        points=np.vstack((ground,roof));plane,on=SurfaceEvidence.measured_ceiling(points,1.2)
        np.testing.assert_allclose(plane,[0,0,2.8],atol=1e-9)
        self.assertEqual(on.sum(),len(roof))

    def test_ceiling_connected_walls_do_not_consume_shelf_back_inspection(self):
        g,d=self.grid();ground=floor_points();ceiling=ground.copy();ceiling[:,2]=2.8
        y,z=np.meshgrid(np.arange(2.,10.,.1),np.arange(.4,2.76,.1))
        wall=np.c_[np.full(y.size,3.),y.ravel(),z.ravel()]
        y,z=np.meshgrid(np.arange(4.,7.,.1),np.arange(.4,1.81,.1))
        front=np.c_[np.full(y.size,7.),y.ravel(),z.ravel()]
        x,y=np.meshgrid(np.arange(7.,7.61,.1),np.arange(4.,7.,.1))
        top=np.c_[x.ravel(),y.ravel(),np.full(x.size,1.8)]
        points=np.vstack((ground,wall,front,top,ceiling));s=SurfaceEvidence()
        g.update_obstacles(np.vstack((wall,front)));d,_=g.search(g.cell((1.,1.)))
        s.missing_geometry_candidates(g,d,points,1.2,limit=64)
        report=s.last_missing_geometry_report
        self.assertGreater(report['ceiling_connected_groups_skipped'],0)
        self.assertGreater(report['visible_boundary_targets'],0)
        self.assertTrue(all(t['xyz'][0]>6. for t in report['targets'] if t['kind']=='boundary'))
        # This is a task veto only: wall geometry still blocks a real ray.
        from ghost_revisit import occupied_voxel_keys,visible_target_count
        self.assertEqual(visible_target_count([1.,5.,1.2],[[5.,5.,1.2]],
            occupied_voxel_keys(points,.2),.2),0)

    def test_sensor_origin_and_tilted_ground_frame_own_the_vertical_cone(self):
        g,d=self.grid();p=floor_points((7.,10.,7.,10.),tilt=(.06,-.04));s=SurfaceEvidence()
        captured=[]
        def ray(view,targets,*args,**kwargs):
            captured.append((np.array(view),np.array(targets[0])));return 1
        with patch('ghost_revisit.visible_target_count',side_effect=ray):
            s.missing_geometry_candidates(g,d,p,1.2,lidar_height=.13)
        self.assertTrue(captured)
        up=np.array([-.06,.04,1.]);up/=np.linalg.norm(up)
        for sensor,target in captured:
            self.assertAlmostEqual(sensor[2],1.2+.13*up[2])
            v=target-sensor;h=np.sqrt(v@v-(v@up)**2)
            elevation=np.degrees(np.arctan2(v@up,h))
            self.assertGreaterEqual(elevation,-20.2)
            self.assertLessEqual(elevation,35.2)

    def test_qualified_top_views_do_not_certify_unmeasured_low_back_face(self):
        g,d=self.grid();ground=floor_points((4.8,6.5,3.8,8.2))
        yy,zz=np.meshgrid(np.arange(4.,8.,.1),np.arange(.4,2.5,.1))
        front=np.c_[np.full(yy.size,5.),yy.ravel(),zz.ravel()]
        xx,yy=np.meshgrid(np.arange(5.,6.21,.1),np.arange(4.,8.,.1))
        top=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,2.4)]
        points=np.vstack((ground,front,top));s=SurfaceEvidence()
        for k in s.keys(points):s.records[tuple(k)]=[8,255,np.array([1.,1.,1.2])]
        g.update_obstacles(front);d,_=g.search(g.cell((1.,1.)))
        s.missing_geometry_candidates(g,d,points,1.2,limit=64)
        requests=[t for t in s.last_missing_geometry_report['targets']
            if t['kind']=='boundary' and t['xyz'][0]>6.1 and t['xyz'][2]<.2]
        self.assertTrue(requests)
        self.assertTrue(any(t['contributed'] for t in requests))

    def grid(self):
        g=ExplorationGrid([0,0,12,12],.2,.8,.4)
        g.known[:]=True;g.update_obstacles([])
        d,_=g.search(g.cell((1.,1.)))
        return g,d

    def test_unseen_floor_hole_creates_safe_views_with_zero_measured_deficit(self):
        g,d=self.grid();p=floor_points((7.,10.,7.,10.));s=SurfaceEvidence()
        # Existing measured surfaces can all be qualified; the actual hole
        # still has NO records and therefore no old-style measured deficit.
        for k in s.keys(p):s.records[tuple(k)]=[5,255,np.array([1.,1.,1.2])]
        self.assertEqual(len(s.deficits(p)[0]),0)
        before=[getattr(g,n).copy() for n in ['known','safe','occupied']]
        mask,gain=s.missing_geometry_candidates(g,d,p,1.2)
        self.assertGreater(s.last_missing_geometry_report['missing_floor_bins'],0)
        self.assertGreater(s.last_missing_geometry_report['visible_floor_targets'],0)
        self.assertTrue(mask.any());self.assertTrue((gain[mask]>0).all())
        self.assertTrue(g.safe[mask].all());self.assertTrue(np.isfinite(d[mask]).all())
        for n,a in zip(['known','safe','occupied'],before):np.testing.assert_array_equal(a,getattr(g,n))

    def test_floor_plane_tracks_current_map_tilt(self):
        p=floor_points(tilt=(.025,-.018))
        np.testing.assert_allclose(SurfaceEvidence.fit_ground_plane(p,1.2),[.025,-.018,0.],atol=1e-10)

    def test_insufficient_plane_evidence_does_not_invent_floor(self):
        g,d=self.grid();s=SurfaceEvidence()
        mask,_=s.missing_geometry_candidates(g,d,np.array([[1.,1.,.1]]),1.2)
        self.assertFalse(mask.any());self.assertIsNone(s.last_missing_geometry_report['floor_plane'])

    def test_downward_blind_cone_cannot_generate_nearby_floor_view(self):
        g,d=self.grid();p=floor_points((4.,7.,4.,7.))
        # Only this tiny patch is observed in the air: all reachable views
        # are closer than the approximately 3.5 m downward-cone requirement.
        g.known[:]=False;g.known[20:35,20:35]=True;g.update_obstacles([])
        d,_=g.search(g.cell((5.,5.)));s=SurfaceEvidence()
        mask,_=s.missing_geometry_candidates(g,d,p,1.2)
        self.assertFalse(mask.any())
        self.assertEqual(s.last_missing_geometry_report['visible_floor_targets'],0)

    def test_shelf_top_anchors_unseen_back_boundary_without_back_points(self):
        g,d=self.grid();ground=floor_points((4.8,6.5,3.8,8.2))
        yy,zz=np.meshgrid(np.arange(4.,8.,.1),np.arange(.4,2.5,.1))
        front=np.c_[np.full(yy.size,5.),yy.ravel(),zz.ravel()]
        xx,yy=np.meshgrid(np.arange(5.,6.21,.1),np.arange(4.,8.,.1))
        top=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,2.4)]
        p=np.vstack((ground,front,top));g.update_obstacles(front)
        d,_=g.search(g.cell((1.,1.)));s=SurfaceEvidence()
        mask,_=s.missing_geometry_candidates(g,d,p,1.2,limit=64)
        self.assertGreater(s.last_missing_geometry_report['unseen_boundary_groups'],0)
        self.assertGreater(s.last_missing_geometry_report['visible_boundary_targets'],0)
        views=np.asarray([g.xy(c) for c in np.argwhere(mask)])
        self.assertTrue((views[:,0]>7.).any())

    def test_complete_low_ceiling_does_not_fill_the_back_face_silhouette(self):
        g,d=self.grid();ground=floor_points((4.8,6.5,3.8,8.2))
        yy,zz=np.meshgrid(np.arange(4.,8.,.1),np.arange(.4,2.5,.1))
        front=np.c_[np.full(yy.size,5.),yy.ravel(),zz.ravel()]
        xx,yy=np.meshgrid(np.arange(5.,6.21,.1),np.arange(4.,8.,.1))
        top=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,2.4)]
        ceiling=floor_points();ceiling[:,2]=2.8
        points=np.vstack((ground,front,top,ceiling));s=SurfaceEvidence()
        g.update_obstacles(front);d,_=g.search(g.cell((1.,1.)))
        mask,_=s.missing_geometry_candidates(g,d,points,1.2,limit=64)
        self.assertIsNotNone(s.last_missing_geometry_report['ceiling_plane'])
        self.assertEqual(s.last_missing_geometry_report['excluded_ceiling_returns'],len(ceiling))
        self.assertGreater(s.last_missing_geometry_report['visible_boundary_targets'],0)
        views=np.asarray([g.xy(c) for c in np.argwhere(mask)])
        self.assertTrue((views[:,0]>7.).any())
        # Roof exclusion is only for silhouette generation; physical sensing
        # still cannot see a target above the actual ceiling.
        from ghost_revisit import occupied_voxel_keys,visible_target_count
        occupied=occupied_voxel_keys(points,.2)
        self.assertEqual(visible_target_count([5.,5.,1.2],[[8.,5.,3.5]],occupied,.2,
            min_elevation_deg=-90.,max_elevation_deg=90.),0)

    def test_small_quota_rotates_through_all_back_boundary_regions(self):
        g,d=self.grid();ground=floor_points((4.8,6.5,1.8,10.2))
        yy,zz=np.meshgrid(np.arange(2.,10.,.1),np.arange(.4,2.5,.1))
        front=np.c_[np.full(yy.size,5.),yy.ravel(),zz.ravel()]
        xx,yy=np.meshgrid(np.arange(5.,6.21,.1),np.arange(2.,10.,.1))
        top=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,2.4)]
        p=np.vstack((ground,front,top));s=SurfaceEvidence()
        g.update_obstacles(front);d,_=g.search(g.cell((1.,1.)))
        seen=set();first=None;total=None
        for _ in range(20):
            s.missing_geometry_candidates(g,d,p,1.2,limit=2)
            report=s.last_missing_geometry_report
            groups={tuple(np.floor(np.asarray(t['xyz'][:2])/2.4).astype(int))
                for t in report['targets'] if t['kind']=='boundary'}
            if first is None:first=groups
            seen.update(groups);total=report['spatial_groups_available']['boundary']
            self.assertLessEqual(sum(t['kind']=='boundary' for t in report['targets']),32)
            if len(seen)==total:break
        self.assertEqual(len(seen),total)
        self.assertGreater(len(seen),len(first))
        # The low-face request exists independently of a high/top return.
        self.assertTrue(any(t['kind']=='boundary' and t['xyz'][2]<.8
            for t in report['targets']))

    def test_opaque_wall_rejects_ground_sightline_even_with_2d_safe_view(self):
        from ghost_revisit import occupied_voxel_keys,visible_target_count
        y,z=np.meshgrid(np.arange(0.,12.,.1),np.arange(0.,2.1,.1))
        wall=np.c_[np.full(y.size,6.),y.ravel(),z.ravel()]
        occupied=occupied_voxel_keys(wall,.2)
        self.assertEqual(visible_target_count([3.,5.,1.2],[[8.,5.,0.]],occupied,.2),0)


if __name__=='__main__':unittest.main()
