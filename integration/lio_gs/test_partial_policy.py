"""An explicit baseline exception must never weaken default strict gates."""
import json
from pathlib import Path
import tempfile
import unittest
from dataset import load


class PartialPolicy(unittest.TestCase):
    def test_both_cli_opt_in_and_dataset_authorization_are_required(self):
        root=Path(__file__).resolve().parents[2]/'half_gs_dataset_runs/dynamic_smoke_20261103_retry2'
        if not (root/'dataset_validation.json').exists():self.skipTest('Actual dynamic smoke dataset unavailable')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            for item in root.iterdir():
                if item.name!='acceptance.json':(p/item.name).symlink_to(item.resolve(),target_is_directory=item.is_dir())
            metadata=dict(dataset_ready=True,geometry_complete=False,joint_complete=False,multiview_missing_samples=None,baseline_training_authorized=True)
            (p/'acceptance.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'geometrically complete'):load(p,96)
            data=load(p,96,allow_partial_geometry=True);self.assertFalse(data['acceptance']['geometry_complete'])
            with self.assertRaisesRegex(ValueError,'Strict photo completeness'):load(p,96,require_complete_photos=True,allow_partial_geometry=True)
            metadata['baseline_training_authorized']=False;(p/'acceptance.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'geometrically complete'):load(p,96,allow_partial_geometry=True)


if __name__=='__main__':unittest.main()
