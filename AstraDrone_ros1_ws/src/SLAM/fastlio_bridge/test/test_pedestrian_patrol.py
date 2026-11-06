import json,sys,tempfile,unittest,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pedestrian_patrol import Patrol
from half_warehouse_scene import prepare
from warehouse_scene import blocked_segment
ROOT=Path(__file__).resolve().parents[5]

class PedestrianPatrolTest(unittest.TestCase):
    def test_exact_sim_duration_no_wall_clock_dependency_and_stays_stopped(self):
        p=Patrol([(1,-12),(5,-12)])
        self.assertEqual(p.velocity(30,(3,-12),False),(0,0));self.assertIsNone(p.started)
        self.assertLess(p.velocity(40,(3,-12),True)[0],0)
        self.assertNotEqual(p.velocity(219.99,(3,-12),True),(0,0))
        self.assertEqual(p.velocity(220,(3,-12),True),(0,0));self.assertEqual(p.finished-p.started,180)
        self.assertEqual(p.velocity(221,(3,-12),True),(0,0))

    def test_route_reverses_without_leaving_clear_segment(self):
        p=Patrol([(1,-12),(5,-12)]);xy=[3.,-12.]
        for n in range(3601):
            vx,vy=p.velocity(1+n*.05,xy,True);xy[0]+=.05*vx;xy[1]+=.05*vy
            self.assertTrue(.95<=xy[0]<=5.05);self.assertAlmostEqual(xy[1],-12)
        self.assertIsNotNone(p.finished)

    def test_optional_actor_yields_away_from_nearby_uav(self):
        p=Patrol([(1,-12),(5,-12)],yield_distance=2.5)
        p.velocity(30,(3,-12),True,(8,-12))
        vx,vy=p.velocity(31,(3,-12),True,(5,-12))
        self.assertTrue(p.yielding)
        self.assertLess(vx,0.)
        self.assertEqual(vy,0.)
        self.assertGreater(p.velocity(32,(3,-12),True,(1.5,-12))[0],0.)
        self.assertEqual(p.velocity(33,(1.01,-12),True,(1.5,-12)),(0.,0.))
        self.assertTrue(p.yielding)

    def test_generated_world_has_two_distinct_models_only_when_enabled(self):
        original=(ROOT/'simulation/astra_gazebo_worlds/cangku.world').read_bytes()
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'case';world=ET.parse(prepare(ROOT,p,True)).find('world')
            names=[i.findtext('name') for i in world.findall('include')]
            self.assertEqual(names.count('0'),1);self.assertEqual(names.count('1'),1)
            meta=json.loads(Path(str(p)+'_scene.json').read_text());spec=meta['extra_pedestrian']
            self.assertEqual(spec['duration_sim_seconds'],180)
            self.assertFalse(blocked_segment(*spec['waypoints_world_xy'][0],*spec['waypoints_world_xy'][1],meta['boxes']))
            self.assertEqual(meta['pedestrian_world'],[-8.,-4.,0.])
            p=Path(d)/'old';world=ET.parse(prepare(ROOT,p)).find('world')
            self.assertFalse(any(i.findtext('name')=='1' for i in world.findall('include')))
        self.assertEqual((ROOT/'simulation/astra_gazebo_worlds/cangku.world').read_bytes(),original)

    def test_invalid_patrol_rejected(self):
        for args in [([],180,.45),([(1,2)],180,.45),([(1,2),(3,4)],-1,.45),([(1,2),(3,4)],180,2)]:
            with self.assertRaises(ValueError):Patrol(*args)

if __name__=='__main__':unittest.main()
