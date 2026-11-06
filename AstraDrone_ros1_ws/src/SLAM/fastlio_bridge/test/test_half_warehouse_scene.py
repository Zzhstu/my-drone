import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from half_warehouse_scene import generate, prepare, corners, SOURCE_SHA256
from warehouse_scene import collision_boxes, pose, blocked, blocked_segment


class HalfWarehouseTest(unittest.TestCase):
    def test_half_area_and_unscaled_racks(self):
        source = ROOT/'simulation/astra_gazebo_worlds/cangku.world'
        before = source.read_bytes()
        original = ET.fromstring(before).find('world')
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'half.world';meta=Path(d)/'scene.json'
            generate(ROOT,p,meta);world=ET.parse(p).getroot().find('world')
            info=json.loads(meta.read_text())
            self.assertEqual(info['nominal_size_m'],[25.25,27.])
            self.assertEqual(info['nominal_area_m2']/info['original_nominal_area_m2'],.5)
            self.assertEqual(len(info['kept_shelves']),24)
            self.assertEqual(len(info['omitted_shelves']),28)
            for name in info['kept_shelves']:
                old=original.find("model[@name='%s']"%name)
                new=world.find("model[@name='%s']"%name)
                # Resource URIs change with output location; the mesh is identical.
                for model,folder in ((old,source.parent),(new,p.parent)):
                    for uri in model.findall('.//geometry/mesh/uri'):
                        uri.text=(folder/uri.text).resolve().as_uri()
                self.assertEqual(ET.tostring(old),ET.tostring(new))
            self.assertEqual(world.findall('state'),[])
        self.assertEqual(source.read_bytes(),before)
        self.assertEqual(hashlib.sha256(before).hexdigest(),SOURCE_SHA256)

    def test_walls_and_roof_match_crop_without_lowering_height(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'half.world';meta=Path(d)/'scene.json'
            generate(ROOT,p,meta);world=ET.parse(p).getroot().find('world')
            shell=world.find("model[@name='Untitled']")
            original=ET.parse(ROOT/'simulation/astra_gazebo_worlds/cangku.world').find("world/model[@name='Untitled']")
            roof=shell.find("link[@name='Floor_0']")
            self.assertEqual(roof.findtext('collision/geometry/box/size'),'25.25 27 0.1')
            self.assertEqual(pose(roof)[2],2.5)
            for link in shell.findall('link'):
                self.assertEqual(link.findtext('collision/geometry/box/size'),
                                 link.findtext('visual/geometry/box/size'))
            for name in ('Wall_0','Wall_4'):
                self.assertEqual(ET.tostring(shell.find("link[@name='%s']"%name)),
                                 ET.tostring(original.find("link[@name='%s']"%name)))
            right=shell.find("link[@name='Wall_2']")
            self.assertAlmostEqual(pose(right)[0],-.075)
            info=json.loads(meta.read_text());bounds=info['bounds_world']
            for box in info['boxes']:
                if box['name'].startswith('shelf'):
                    xy=corners(box)
                    self.assertTrue(np.all(xy[:,0]<bounds[2]-.15))
                    self.assertTrue(np.all(xy[:,0]>bounds[0]+.15))

    def test_runtime_resources_resolve_and_person_is_added_safely(self):
        with tempfile.TemporaryDirectory() as d:
            prefix=Path(d)/'case'
            p=prepare(ROOT,prefix);world=ET.parse(p).getroot().find('world')
            people=[node for node in world.findall('include') if node.findtext('name')=='0']
            self.assertEqual(len(people),1)
            self.assertEqual(people[0].findtext('uri'),'model://person_standing')
            self.assertEqual(people[0].findtext('pose'),'-8 -4 0 0 0 0')
            for uri in world.findall('.//geometry/mesh/uri'):
                self.assertTrue(uri.text.startswith('file://'))
                self.assertTrue(Path(uri.text[len('file://'):]).is_file())
            info=json.loads(Path(str(prefix)+'_scene.json').read_text())
            self.assertEqual(info['pedestrian_world'],[-8.,-4.,0.])
            self.assertFalse(blocked(-8,-4,info['boxes'],.35))
            self.assertFalse(blocked_segment(-8,-4,-8,-7,info['boxes']))
            self.assertFalse(blocked_segment(-8,-7,-8,-1,info['boxes']))

    def test_launch_keeps_unknown_geometry_exploration(self):
        p=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/half_warehouse_exploration.launch'
        text=p.read_text()
        self.assertIn('warehouse_exploration.launch',text)
        self.assertIn('small_indoor_velocity.py',text)
        # User-approved CPA is 0.40m horizontal envelope + 0.20m to the
        # detected centre. It is independent of the 0.80m planning clearance.
        emergency=ET.parse(p).getroot().find("param[@name='/collision_risk/emergency_cpa_distance']")
        self.assertAlmostEqual(float(emergency.attrib['value']),.4+.2)
        params={e.attrib['name']:float(e.attrib['value']) for e in ET.parse(p).getroot().findall('param')
                if e.attrib['name'].endswith(('max_navigation_speed','max_total_speed'))}
        self.assertGreaterEqual(params['/mavros_avoidance_controller/max_total_speed'],
                                params['/mavros_avoidance_controller/max_navigation_speed'])
        self.assertNotIn('bounds_world',text)
        self.assertNotIn('scene_file',text)


if __name__ == '__main__':
    unittest.main()
