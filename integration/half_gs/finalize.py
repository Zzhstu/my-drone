"""Record a separately culled model without inheriting the parent's receipts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def run(path):
    root = Path(path).resolve()
    read = lambda p: json.loads(p.read_text())
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    training = read(root / 'gsplat/training_result.json')
    exports = read(root / 'gsplat/export_validation.json')
    policy = read(root / 'gsplat/semantic_culling.json')
    if not training['trained'] or not exports['passed'] or sha(root / 'gsplat/model.ply') != exports['model_sha256']:
        raise ValueError('Candidate exports have not been verified')
    if not training['input_unchanged'] or not training['source_unchanged'] or policy['truth_used']:
        raise ValueError('Invalid candidate provenance')
    snapshot = root / 'gsplat' / policy['source_snapshot']
    if sha(snapshot) != policy['source_sha256']:
        raise ValueError('Semantic culling source archive changed')
    receipt = read(root / 'parent_acceptance.json')
    receipt.update(completed=True, output=str(root), training=training, export_validation=exports,
        semantic_gaussian_culling=policy, parent_model=policy['parent'],
        scope='Actual parent SITL/12000-step training followed by image-vote Gaussian culling. '
              'Input coverage gaps retained. No complete dynamic-removal or real-flight claim.',
        finalizer_sha256=sha(Path(__file__)))
    shutil.copyfile(__file__, root / 'gsplat/finalizer_source.py')
    (root / 'acceptance.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(dict(completed=True, model=str(root), gaussians=exports['gaussians'],
                         joint_complete=receipt['strict_joint_complete'])))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('model'); run(p.parse_args().model)
