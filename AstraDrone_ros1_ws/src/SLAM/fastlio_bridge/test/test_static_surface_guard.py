"""Exercise the exact production Eigen helper, including rotated geometry."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class StaticSurfaceGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix='astra_surface_guard_')
        cls.binary=str(Path(cls.temp.name)/'geometry')
        package=Path(__file__).resolve().parents[1]
        subprocess.run(['c++','-std=c++14','-O2','-I/usr/include/eigen3',
            '-I'+str(package/'include'),str(package/'test/test_static_surface_guard.cpp'),
            '-o',cls.binary],check=True,capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_rotated_preexisting_wall(self):
        subprocess.run([self.binary,'1'],check=True)

    def test_narrow_person_is_not_structure(self):
        subprocess.run([self.binary,'2'],check=True)

    def test_line_is_not_surface(self):
        subprocess.run([self.binary,'3'],check=True)

    def test_floor_does_not_hide_person(self):
        subprocess.run([self.binary,'4'],check=True)

    def test_eight_cm_offset_remains_suspect(self):
        subprocess.run([self.binary,'5'],check=True)

    def test_insufficient_support_remains_suspect(self):
        subprocess.run([self.binary,'6'],check=True)

    def test_wall_floor_junction(self):
        subprocess.run([self.binary,'7'],check=True)
