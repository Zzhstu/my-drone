import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from resources import ResourceMeter


class ResourceLimits(unittest.TestCase):
    def test_real_file_budget_is_detected(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'sensors.bag.active').write_bytes(b'x'*4096)
            meter=ResourceMeter(folder,run_gib=1.,bag_gib=1e-7,reserve_gib=0.)
            value=meter.sample()
            self.assertEqual(value['bag_bytes'],4096)
            self.assertIn('bag_bytes budget exceeded',value['reason'])

    def test_child_memory_is_counted_and_released(self):
        with tempfile.TemporaryDirectory() as folder:
            meter=ResourceMeter(folder,reserve_gib=0.)
            before=meter.sample()['process_tree_rss_bytes']
            child=subprocess.Popen([sys.executable,'-c',
                'import time; a=bytearray(50*1024**2); print("ready",flush=True); time.sleep(30)'],stdout=subprocess.PIPE,text=True)
            try:
                self.assertEqual(child.stdout.readline().strip(),'ready')
                during=meter.sample()['process_tree_rss_bytes']
                self.assertGreater(during-before,45*2**20)
                meter.limits['process_tree_rss_bytes']=before+20*2**20
                self.assertIn('process_tree_rss_bytes budget exceeded',meter.sample()['reason'])
            finally:
                child.terminate();child.wait();child.stdout.close()
            self.assertLess(meter.sample()['process_tree_rss_bytes']-before,10*2**20)


if __name__=='__main__':unittest.main()
