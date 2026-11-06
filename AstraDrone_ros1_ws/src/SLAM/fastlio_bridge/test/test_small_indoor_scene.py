"""Scene migration checks: geometry, pinned assets, isolated runtime copy."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from small_indoor_scene import prepare,interior_wall_samples


class SmallIndoorSceneTest(unittest.TestCase):
    def setUp(self):
        self.folder=ROOT/'simulation/astra_gazebo_worlds/small_office'
        self.source=self.folder/'office_small.original.world'
        self.meta=json.loads((self.folder/'SOURCE.json').read_text())

    def test_pinned_meshes_have_no_missing_resources(self):
        for item in self.meta['assets']:
            p=self.folder/item['path']
            self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),item['sha256'])
            self.assertEqual(len(ET.parse(p).getroot().findall('.//{*}image')),0)

    def test_selected_collision_geometry_preserved(self):
        before=self.source.read_bytes()
        original=ET.fromstring(before).find('world')
        with tempfile.TemporaryDirectory() as d:
            output=prepare(ROOT,Path(d)/'case')
            world=ET.parse(output).getroot().find('world')
            self.assertEqual(len(world.findall('state')),0)
            self.assertEqual({m.get('name') for m in world.findall('model')},set(self.meta['kept_model_names']))
            for model in world.findall('model'):
                old=original.find("model[@name='%s']"%model.get('name'))
                self.assertEqual(model.findtext('pose'),old.findtext('pose'))
                collisions=model.findall('.//collision');sources=old.findall('.//collision')
                self.assertEqual(len(collisions),len(sources))
                for a,b in zip(collisions,sources):
                    self.assertEqual(a.findtext('pose'),b.findtext('pose'))
                    self.assertEqual(a.findtext('geometry/box/size'),b.findtext('geometry/box/size'))
                    mesh=a.findtext('geometry/mesh/uri')
                    if mesh:
                        self.assertTrue(Path(mesh[len('file://'):]).is_file())
                        self.assertTrue(mesh.endswith(b.findtext('geometry/mesh/uri')[len('model://'):]))
            self.assertEqual(world.findtext('physics/max_step_size'),'0.001')
            self.assertEqual(world.findtext('physics/real_time_update_rate'),'1000')
            self.assertEqual(world.findall('.//material/script'),[])
        self.assertEqual(self.source.read_bytes(),before)

    def test_wall_reference_is_inward_and_excludes_ground(self):
        points=interior_wall_samples(self.source)
        self.assertEqual(points.shape[1],3)
        self.assertTrue(np.isfinite(points).all())
        self.assertGreater(len(points),1000)
        self.assertGreaterEqual(points[:,2].min(),.25)
        self.assertLessEqual(points[:,2].max(),2.20)
        # Original shell center is (-.009790,-.061782); inward faces +/-4.85m.
        xy=points[:,:2]-[-.009790,-.061782]
        self.assertTrue(np.all(np.max(np.abs(xy),axis=1)<4.851))
        self.assertTrue(np.all(np.max(np.abs(xy),axis=1)>4.849))

    def test_launch_does_not_supply_world_geometry_to_explorer(self):
        launch=(ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/small_indoor_exploration.launch').read_text()
        self.assertNotIn('scene_file',launch)
        self.assertNotIn('bounds',launch)
        self.assertIn('warehouse_exploration.launch',launch)


if __name__=='__main__':
    unittest.main()
