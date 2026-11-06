"""Independent fixed real-mesh audit, including masks; never optimizer input."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'integration/roomgs'))
from integration.roomgs.warehouse20_audit import intersector,view_mask,has_translation_baseline


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate_capture_camera(root,definition,info):
    root=Path(root);path=root/'camera_profile.json';k=np.array(info['K']).reshape(3,3)
    optics=None
    if path.exists():
        manifest=json.loads((root/'source_manifest.json').read_text())
        if manifest.get(str(path.resolve()))!=sha(path):raise ValueError('Recorded capture camera profile changed')
        optics=json.loads(path.read_text())['optics']
    expected=optics['K'] if optics else definition['camera_K']
    if not np.allclose(k,np.array(expected),atol=1e-3 if optics else 1e-5,rtol=0):
        raise ValueError('Actual camera intrinsics do not match recorded capture optics')
    if optics and (info['width']!=optics['width'] or info['height']!=optics['height']):
        raise ValueError('Actual camera image dimensions differ from recorded capture optics')
    return optics


def run(root):
    root=Path(root);definition=json.loads((root/'target_manifest.json').read_text())
    if sha(root/'mesh_reference.npz')!=definition['reference_sha256']:raise ValueError('Frozen audit targets changed')
    if sha(root/'scene_cangku_half.world')!=definition['world_sha256']:raise ValueError('Runtime world does not match frozen reference')
    data=np.load(root/'mesh_reference.npz');p,n,labels=[data[k] for k in ['points','normals','labels']];ray=intersector(data['triangles'])
    static=json.loads((root/'scene_scene.json').read_text()).get('actors')==[]
    if static and not json.loads((root/'result.json').read_text()).get('actor_free_verified'):
        raise ValueError('Static audit requires actual actor-absence verification')
    alignment=np.array(json.loads((root/'audit_alignment.json').read_text())['world_from_estimator'])
    rows=json.loads((root/'dataset/frames.json').read_text());info=json.loads((root/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    capture_optics=validate_capture_camera(root,definition,info)
    count=np.zeros(len(p),np.int32);baseline=np.zeros(len(p),bool);first=np.full(p.shape,np.nan);masks=[];centers=[]
    for i,row in enumerate(rows):
        pose=alignment@np.array(row['c2w']);seen=view_mask(ray,p,n,pose,k,info['width'],info['height'])
        ids=np.flatnonzero(seen)
        if len(ids) and not static:
            camera=(p[ids]-pose[:3,3])@pose[:3,:3];uv=camera[:,:2]/camera[:,2,None]
            pix=(uv*np.array([k[0,0],k[1,1]])+k[:2,2]).astype(int)
            path=root/'dataset/masks'/(Path(row['file_path']).name+'.png')
            valid=np.asarray(Image.open(path));seen[ids]&=valid[pix[:,1],pix[:,0]]>127
        new=seen&(count==0);first[new]=pose[:3,3];count+=seen;ids=np.flatnonzero(seen)
        baseline[ids]|=np.linalg.norm(first[ids]-pose[:3,3],axis=1)>=.4
        masks.append(np.packbits(seen));centers.append(pose[:3,3])
        if i%200==0:print('Actual fixed-target audit',i,len(rows),flush=True)
    packed=np.array(masks);centers=np.array(centers)
    for i in np.flatnonzero((count>=2)&~baseline):
        visible=((packed[:,i//8]>>(7-i%8))&1).astype(bool)
        baseline[i]=has_translation_baseline(centers[visible])
    xyz=np.load(root/'static_map.npz')['xyz'];world=xyz@alignment[:3,:3].T+alignment[:3,3]
    distances=cKDTree(world).query(p,workers=2)[0];geometry=distances<=.2;photos=(count>=2)&baseline;complete=geometry&photos
    report=dict(passed=bool(complete.all()),required_fraction=1.,total_samples=len(p),missing_samples=int((~complete).sum()),
        geometry_fraction=float(geometry.mean()),multiview_fraction=float(photos.mean()),complete_fraction=float(complete.mean()),
        geometry_missing_samples=int((~geometry).sum()),multiview_missing_samples=int((~photos).sum()),step_m=.2,distance_tolerance_m=.2,
        minimum_views=2,minimum_translation_baseline_m=.4,scope=definition['scope'],
        truth_usage='Frozen real-mesh targets and single initial alignment OFFLINE audit only; not a point/pose/mask training source',
        dynamic_visibility_scope='Actor-free static world, implicit all-valid image masks' if static else 'Learned image masks exclude predicted people; missed detections may overestimate dynamic occlusion coverage',
        groups={str(label):dict(total=int((labels==label).sum()),geometry=int(geometry[labels==label].sum()),multiview=int(photos[labels==label].sum()),complete=int(complete[labels==label].sum())) for label in sorted(set(labels))},
        capture_camera_K=k.tolist(),eligibility_camera_K=definition['camera_K'],
        camera_profile=capture_optics,
        camera_audit_scope='Actual recorded camera optics evaluate the unchanged reference target set; old wider-FOV eligibility is not recomputed or shrunk')
    (root/'coverage.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(root/'coverage_samples.npz',points=p,normals=n,labels=labels,nearest_map_distance=distances,views=count,baseline=baseline,complete=complete)
    np.savez_compressed(root/'missing_surfaces.npz',points=p[~complete],normals=n[~complete],labels=labels[~complete],geometry=geometry[~complete],photos=photos[~complete])
    validation=json.loads((root/'dataset_validation.json').read_text());capture=json.loads((root/'result.json').read_text())
    authorization=json.loads((root/'inspection_plan.json').read_text()).get('partial_baseline_authorized') is True
    acceptance=dict(dataset_ready=bool(validation['passed'] and capture['success'] and capture['source_unchanged']),geometry_complete=bool(geometry.all()),
        joint_complete=bool(complete.all()),multiview_missing_samples=report['multiview_missing_samples'],baseline_training_authorized=authorization,
        mask_source='Implicit all-valid in verified actor-free world' if static else 'Image-only YOLO11s-seg; no simulator truth labels',scope=(('Static multi-height major-region baseline' if static else 'User chose major-region dynamic-removal baseline')+'; coverage gaps retained, strict100% gate not declared passed' if authorization else 'Captured data and fixed-target audit only; no explicit partial-training authorization in route'))
    (root/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');run(p.parse_args().dataset)
