#!/usr/bin/env python3
"""Check that the different world does not shrink furniture or obstruct starts."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from office50_scene import prepare
from warehouse_scene import blocked_segment
from pedestrian_patrol import Patrol


class OfficeSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory()
        cls.prefix=Path(cls.temp.name)/'scene'
        cls.root=Path(__file__).resolve().parents[5]
        cls.world=prepare(cls.root,cls.prefix)
        cls.scene=json.loads(Path(str(cls.prefix)+'_scene.json').read_text())

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_room_area_and_native_furniture_scale(self):
        self.assertAlmostEqual(self.scene['nominal_area_m2'],50.4)
        self.assertEqual(self.scene['static_furniture_count'],12)
        desk=next(x for x in self.scene['furniture'] if x['name']=='desk_nw')
        self.assertAlmostEqual(desk['size'][0],1.8045,places=4)
        self.assertAlmostEqual(desk['size'][2],.7219,places=4)

    def test_actor_route_is_open_and_one_minute(self):
        spec=self.scene['extra_pedestrian'];points=spec['waypoints_world_xy']
        self.assertFalse(blocked_segment(*points[0],*points[1],self.scene['boxes'],.35))
        patrol=Patrol(points,spec['duration_sim_seconds'],spec['speed_mps'])
        patrol.velocity(20.,points[0],True);self.assertIsNone(patrol.finished)
        patrol.velocity(79.95,points[0],True);self.assertIsNone(patrol.finished)
        self.assertEqual(patrol.velocity(80.,points[0],True),(0.,0.))
        self.assertEqual(patrol.finished-patrol.started,60.)

    def test_resolved_assets_and_single_pose_owner_target(self):
        world=ET.parse(self.world).getroot().find('world')
        for uri in world.findall('.//geometry/mesh/uri'):
            self.assertTrue(uri.text.startswith('file://'))
            self.assertTrue(Path(uri.text[len('file://'):]).is_file())
        self.assertEqual([x.findtext('name') for x in world.findall('include')],['1'])
        self.assertEqual(len(world.findall('state')),0)

    def test_static_layout_keeps_west_return_corridor_outside_actor_sweep(self):
        self.assertFalse(blocked_segment(-.7,-1.8,-.7,1.3,self.scene['boxes'],.95))
        route=self.scene['extra_pedestrian']['waypoints_world_xy']
        self.assertGreater(min(abs(p[0]+.7) for p in route),1.7)


if __name__=='__main__':unittest.main()
