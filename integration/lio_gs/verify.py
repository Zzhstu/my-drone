"""Post-training export and fixed-surface audit, separate from optimization."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from plyfile import PlyData
from scipy.spatial import cKDTree
import imageio.v2 as imageio


def verify(output):
    output=Path(output);folder=output/'gsplat';data=np.load(folder/'gaussians.npz')
    ply=PlyData.read(folder/'model.ply')['vertex'].data
    expected=dict(means=np.column_stack([ply[n] for n in ['x','y','z']]),
        scales=np.column_stack([ply['scale_%d'%i] for i in range(3)]),
        quats=np.column_stack([ply['rot_%d'%i] for i in range(4)]),opacities=ply['opacity'],
        sh0=np.column_stack([ply['f_dc_%d'%i] for i in range(3)])[:,None,:],
        shN=np.column_stack([ply['f_rest_%d'%i] for i in range(45)]).reshape(-1,3,15).transpose(0,2,1))
    differences={name:float(np.max(np.abs(values-data[name]))) for name,values in expected.items()}
    if any(value>1e-6 for value in differences.values()):raise ValueError('PLY/NPZ parameter mismatch')
    if not all(np.isfinite(data[name]).all() for name in expected):raise ValueError('Non-finite model')
    if np.min(np.linalg.norm(data['quats'],axis=1))<1e-6:raise ValueError('Degenerate rotation')
    source=json.loads((output/'source_dataset.json').read_text());root=Path(source['path'])
    input_unchanged=all(hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest for name,digest in source['input_sha256'].items())
    if not input_unchanged:raise ValueError('Measured input changed')
    opacity=1/(1+np.exp(-data['opacities']));active=data['means'][opacity>=.01]
    if not len(active):raise ValueError('No visible learned Gaussians')
    alignment=np.array(json.loads((root/'audit_alignment.json').read_text())['world_from_estimator'])
    reference=np.load(root/'coverage_samples.npz');points=reference['points']
    world=active@alignment[:3,:3].T+alignment[:3,3];distance=cKDTree(world).query(points)[0];supported=distance<=.2
    coverage=json.loads((root/'coverage.json').read_text())
    geometry=dict(geometry_passed=bool(supported.all()),active_gaussians=len(active),total_samples=len(points),supported_samples=int(supported.sum()),
        missing_samples=int((~supported).sum()),distance_tolerance_m=.2,
        input_photo_missing=coverage.get('multiview_missing_samples',coverage['missing_samples']),
        input_joint_missing=coverage['missing_samples'],joint_complete=bool(supported.all() and coverage['passed']),
        scope='Independent post-training fixed 20cm externally observable targets only; reference/truth never optimizer input')
    (folder/'model_coverage.json').write_text(json.dumps(geometry,indent=2)+'\n')
    video=imageio.get_reader(folder/'walkthrough.mp4');frames=video.count_frames();metadata=video.get_meta_data();video.close()
    if frames!=180:raise ValueError('Video frame count mismatch')
    metrics=json.loads((folder/'training_result.json').read_text())
    if not metrics['trained'] or not metrics['input_unchanged'] or not metrics['source_unchanged']:raise ValueError('Training did not finish with stable sources')
    if metrics['same_eight_final_psnr']<=metrics['same_eight_initial_psnr']:raise ValueError('Same-view holdout did not improve')
    report=dict(passed=True,gaussians=len(ply),parameter_max_abs_differences=differences,input_unchanged=input_unchanged,
        finite=True,sh_rest_norm=float(np.linalg.norm(data['shN'])),video_frames=frames,video_fps=metadata['fps'],
        model_sha256=hashlib.sha256((folder/'model.ply').read_bytes()).hexdigest(),geometry=geometry,
        scope='Actual learned PLY/NPZ parameters and video validation; geometry/photo completeness kept separate')
    (folder/'export_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output');verify(parser.parse_args().output)
