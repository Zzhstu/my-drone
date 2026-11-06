"""Compile the production evidence helper and exercise actual ray geometry."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class ObservedFreeMotionTests(unittest.TestCase):
    def test_geometric_and_temporal_evidence(self):
        package=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'observed_free_motion_test'
            subprocess.run(['g++','-std=c++14','-O2','-UNDEBUG',
                '-I'+str(package/'include'),'-I/usr/include/eigen3',
                str(Path(__file__).with_suffix('.cpp')),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)


if __name__=='__main__':
    unittest.main()
