"""OFFLINE post-training validation only; never imported by the optimizer."""
import argparse
from pathlib import Path
import json
import hashlib
import numpy as np
from scipy.spatial import cKDTree
from geometry import transform


def main(output):
    output=Path(output);data=np.load(output/'gsplat/gaussians.npz')
    alpha=1/(1+np.exp(-data['opacities']))
    active=data['means'][alpha>=.01]
    alignment=np.array(json.loads((output/'audit_alignment.json').read_text())['world_from_estimator'])
    reference=np.load(output/'coverage_samples.npz')
    distances,_=cKDTree(transform(active,alignment)).query(reference['points'])
    supported=distances<=.20
    coverage=json.loads((output/'coverage.json').read_text())
    report=dict(passed=bool(supported.all() and coverage['passed']),
        active_gaussians=len(active),minimum_opacity=.01,total_samples=len(supported),
        supported_samples=int(supported.sum()),complete_fraction=float(supported.mean()),
        missing_samples=int((~supported).sum()),distance_tolerance_m=.20,
        scope='Same predefined finite-resolution surface audit, after training; never optimizer input',
        model_sha256=hashlib.sha256((output/'gsplat/model.ply').read_bytes()).hexdigest())
    (output/'gsplat/model_coverage.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:raise RuntimeError('Learned Gaussian means failed surface support audit; retained model is NOT a fully verified result')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',required=True);main(p.parse_args().capture)
