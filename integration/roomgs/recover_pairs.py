"""Recover existing exact bag observations after a fully completed flight only."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]


def recover(output):
    output=Path(output)
    result=json.loads((output/'result.json').read_text())
    coverage=json.loads((output/'coverage.json').read_text())
    if not (result.get('scene')=='warehouse20' and result.get('landed') and result.get('disarmed')
            and result.get('flight_finished') and result.get('source_unchanged')
            and coverage['geometry_fraction']==1. and coverage['missing_samples']>0
            and result.get('input_status',{}).get('errors',1)==0):
        raise RuntimeError('Exact pair recovery requires a complete, healthy flight and full measured geometry')
    if (output/'dataset_extension.json').exists():raise RuntimeError('Dataset already extended; inspect existing evidence')
    backup=output/'before_bag_extension_audit';backup.mkdir(exist_ok=True)
    for name in ('result.json','coverage.json','coverage_samples.npz','missing_surfaces.json'):
        if not (backup/name).exists():shutil.copyfile(output/name,backup/name)
    helper=ROOT/'integration/roomgs'
    prefix=ROOT/'.envs/roomgs';env=os.environ.copy();env.pop('PYTHONPATH',None)
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['OPENBLAS_NUM_THREADS']='1'
    if not (output/'dataset/native_optical_poses.json').exists():
        subprocess.run(['/usr/bin/python3',str(helper/'dataset_bag_tools.py'),'poses',str(output)],check=True)
    # This raises before image/index changes if any target lacks real evidence.
    subprocess.run([str(prefix/'bin/python'),str(helper/'warehouse20_select_pairs.py'),str(output)],env=env,check=True)
    subprocess.run(['/usr/bin/python3',str(helper/'dataset_bag_tools.py'),'images',str(output),
                    '--stamps',str(output/'additional_pair_selection.json')],check=True)
    subprocess.run([str(prefix/'bin/python'),str(helper/'recheck.py'),str(output)],env=env,check=True)


if __name__=='__main__':recover(sys.argv[1])
