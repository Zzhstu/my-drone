"""Self-contained measured-data bundle, using rigid LiDAR registration only.

The primary map and its completed flight remain unchanged. Supplemental
successful native optical measurements may come from a navigation-aborted,
normally landed flight; its original failure is retained explicitly.
"""
from pathlib import Path
import argparse,json,os,shutil
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from geometry import read_pcd,transform
from provenance import sha,verify_sources
from registration import register,voxel
from warehouse20_audit import audit,intersector,view_mask,has_translation_baseline,write


def link(source,dest):
    dest=Path(dest);dest.parent.mkdir(parents=True,exist_ok=True)
    # Measurement images are immutable and dominate storage. Keep mutable
    # metadata/native-map records independent of later source-run rechecks.
    if dest.suffix.lower() not in ('.jpg','.jpeg','.png'):
        shutil.copyfile(source,dest);return
    try:os.link(source,dest)
    except OSError:shutil.copyfile(source,dest)


def read(path):return json.loads(Path(path).read_text())


def healthy(source,primary=False):
    result=read(source/'result.json');verify_sources(source,result)
    if not (result.get('source_unchanged') and result.get('landed') and result.get('disarmed')
            and result.get('input_status',{}).get('errors',1)==0 and result.get('visual_updates',0)>10):
        raise RuntimeError('Source lacks verified healthy measurements and normal landing: '+str(source))
    if primary and not result.get('flight_finished'):raise RuntimeError('Primary inspection flight did not finish')
    if sha(source/'rgb_pt.pcd')!=result['rgb_map_sha256']:raise RuntimeError('Native source map changed')
    if sha(source/'dataset/frames.json')!=result['coverage']['input_sha256']['dataset/frames.json']:
        raise RuntimeError('Original measured frame index changed')
    return result


def create(primary,output):
    result=healthy(primary,primary=True)
    if result['coverage']['geometry_fraction']!=1.:raise RuntimeError('Primary must already have full measured geometry')
    output.mkdir(parents=True,exist_ok=False)
    names=['rgb_pt.pcd','camera_input.json','sensor_geometry.json','audit_scene.json','target_manifest.json',
           'mesh_reference.npz','audit_alignment.json','acquisition_sources.json']
    for name in names:link(primary/name,output/name)
    shutil.copytree(primary/'acquisition_sources',output/'acquisition_sources',copy_function=shutil.copyfile)
    for src,dst in [('result.json','result_initial.json'),('result.json','native_primary_result.json'),
                    ('coverage.json','coverage_initial.json'),('dataset/frames.json','dataset/frames_initial.json')]:
        link(primary/src,output/dst)
    rows=read(primary/'dataset/frames.json')
    for row in rows:
        source=primary/'dataset'/row['file_path']
        if sha(source)!=row['sha256']:raise RuntimeError('Primary image changed')
        link(source,output/'dataset'/row['file_path'])
    write(output/'dataset/frames.json',rows)
    (output/'dataset/frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    manifest=dict(primary_source=str(primary.resolve()),primary_result_sha256=sha(primary/'result.json'),
        primary_native_map_sha256=sha(primary/'rgb_pt.pcd'),primary_frames_sha256=sha(primary/'dataset/frames.json'),
        primary_frame_count=len(rows),sessions=[],truth_used_for_registration=False,
        scope='Primary completed inspection and immutable native map; real supplemental images rigidly registered from measured LiDAR maps; not a single-sortie claim')
    write(output/'source_sessions.json',manifest)
    result.update(success=False,output=str(output.resolve()),mode='offline_measured_multisession',
        reconstruction_scope=manifest['scope'],primary_flight_finished=True)
    write(output/'result.json',result)
    audit(output)


def add(source,output):
    original=healthy(source);manifest=read(output/'source_sessions.json')
    identity=source.name
    if any(s['id']==identity for s in manifest['sessions']):raise RuntimeError('Session already added')
    base_info=read(output/'camera_input.json');source_info=read(source/'camera_input.json')
    if not (np.allclose(base_info['K'],source_info['K'],atol=1e-8) and
            base_info['width']==source_info['width'] and base_info['height']==source_info['height']
            and all(abs(x)<1e-8 for x in source_info['D'])):raise RuntimeError('Supplemental camera calibration differs')
    definition=read(output/'target_manifest.json');other=read(source/'target_manifest.json')
    for key in ('world_sha256','mesh_sha256','reference_sha256'):
        if definition[key]!=other[key]:raise RuntimeError('Different scene/targets: '+key)
    folder=output/'sessions'/identity;folder.mkdir(parents=True)
    for name in ['result.json','rgb_pt.pcd','camera_input.json','sensor_geometry.json','target_manifest.json','acquisition_sources.json','dataset/frames.json']:
        link(source/name,folder/name)
    shutil.copytree(source/'acquisition_sources',folder/'acquisition_sources',copy_function=shutil.copyfile)
    matrix=register(folder/'rgb_pt.pcd',output/'rgb_pt.pcd',folder/'registration.json')
    data=np.load(output/'mesh_reference.npz');coverage=np.load(output/'coverage_samples.npz')
    missing=np.flatnonzero(~coverage['complete']);points=data['points'][missing];normals=data['normals'][missing]
    ray=intersector(data['triangles']);alignment=np.array(read(output/'audit_alignment.json')['world_from_estimator'])
    k=np.array(base_info['K']).reshape(3,3);width=base_info['width'];height=base_info['height']
    rows=read(output/'dataset/frames.json');candidates=read(source/'dataset/frames.json')
    def seen(pose):return view_mask(ray,points,normals,alignment@pose,k,width,height)
    centers=np.array([(alignment@np.array(r['c2w']))[:3,3] for r in rows])
    masks=np.array([seen(np.array(r['c2w'])) for r in rows])
    poses=np.array([matrix@np.array(r['c2w']) for r in candidates])
    candidate_centers=np.array([(alignment@p)[:3,3] for p in poses]);candidate_masks=np.array([seen(p) for p in poses])
    selected=set()
    def choose(index):
        nonlocal centers,masks
        index=int(index)
        if index in selected:return
        selected.add(index);centers=np.vstack([centers,candidate_centers[index]])
        masks=np.vstack([masks,candidate_masks[index]])
    for column in range(len(missing)):
        existing=centers[masks[:,column]]
        if has_translation_baseline(existing):continue
        ids=np.flatnonzero(candidate_masks[:,column])
        if not len(ids):continue
        if len(existing):
            distances=np.linalg.norm(candidate_centers[ids,None]-existing[None],axis=-1)
            best=np.unravel_index(np.argmax(distances),distances.shape)
            if distances[best]>=.4-1e-9:choose(ids[best[0]]);continue
        xyz=candidate_centers[ids];distances=np.linalg.norm(xyz[:,None]-xyz[None,:],axis=-1)
        a,b=np.unravel_index(np.argmax(distances),distances.shape)
        choose(ids[a]);choose(ids[b])  # Genuine partial witnesses can help the next session.
    for index in sorted(selected):
        old=candidates[index];src=source/'dataset'/old['file_path']
        if sha(src)!=old['sha256']:raise RuntimeError('Supplemental image changed')
        relative=Path('images_sources')/identity/Path(old['file_path']).name
        link(src,output/'dataset'/relative)
        rows.append(dict(old,file_path=str(relative),c2w=poses[index].tolist(),source_session=identity,
            source_frame_index=index,pose_transform='rigid measured LiDAR registration; original optical measurement stamp unchanged'))
    entry=dict(id=identity,original_source=str(source.resolve()),metadata_folder=str(folder.relative_to(output)),
        original_result_sha256=sha(folder/'result.json'),original_frames_sha256=sha(folder/'dataset/frames.json'),
        native_map_sha256=sha(folder/'rgb_pt.pcd'),registration_sha256=sha(folder/'registration.json'),
        selected_source_indices=sorted(selected),selected_images=len(selected),
        original_flight_result=original.get('flight_result'),original_flight_finished=original.get('flight_finished'),
        observation_scope='Successful native VIO poses and exact images from a normally landed source; original navigation failure retained')
    manifest['sessions'].append(entry)
    tools=output/'session_tools';tools.mkdir(exist_ok=True)
    for path in [Path(__file__).resolve(),Path(__file__).with_name('registration.py').resolve()]:
        digest=sha(path);dest=tools/digest
        if not dest.exists():shutil.copyfile(path,dest)
    write(output/'source_sessions.json',manifest);write(output/'dataset/frames.json',rows)
    (output/'dataset/frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    report=audit(output)
    print(json.dumps(dict(added_source=identity,added_images=len(selected),remaining=report['missing_samples'],
        native_primary_map_unchanged=sha(output/'rgb_pt.pcd')==manifest['primary_native_map_sha256']),indent=2),flush=True)


def verify_bundle(output):
    manifest=read(output/'source_sessions.json')
    if sha(output/'native_primary_result.json')!=manifest['primary_result_sha256']:raise RuntimeError('Primary record changed')
    if sha(output/'rgb_pt.pcd')!=manifest['primary_native_map_sha256']:raise RuntimeError('Native primary map changed')
    if sha(output/'dataset/frames_initial.json')!=manifest['primary_frames_sha256']:raise RuntimeError('Primary pose index changed')
    rows=read(output/'dataset/frames.json');initial=read(output/'dataset/frames_initial.json')
    if rows[:len(initial)]!=initial:raise RuntimeError('Primary optical measurements were altered')
    registry={}
    for entry in manifest['sessions']:
        folder=output/entry['metadata_folder'];result=read(folder/'result.json');verify_sources(folder,result)
        if sha(folder/'result.json')!=entry['original_result_sha256'] or sha(folder/'dataset/frames.json')!=entry['original_frames_sha256']:
            raise RuntimeError('Source acquisition metadata changed')
        if not (result['source_unchanged'] and result['landed'] and result['disarmed'] and result['input_status']['errors']==0):
            raise RuntimeError('Supplemental source health invalid')
        registration=read(folder/'registration.json');matrix=np.array(registration['target_from_source'])
        if sha(folder/'registration.json')!=entry['registration_sha256'] or not registration['passed']:
            raise RuntimeError('Registration record changed/failed')
        if sha(folder/'rgb_pt.pcd')!=entry['native_map_sha256'] or entry['native_map_sha256']!=registration['source_sha256']:
            raise RuntimeError('Supplemental native geometry changed')
        if registration['target_sha256']!=manifest['primary_native_map_sha256']:raise RuntimeError('Different registration target')
        if not (np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-8) and abs(np.linalg.det(matrix[:3,:3])-1)<1e-8):
            raise RuntimeError('Nonrigid/scale pose transform rejected')
        a=voxel(read_pcd(folder/'rgb_pt.pcd')[0]);b=voxel(read_pcd(output/'rgb_pt.pcd')[0]);test=a[np.arange(len(a))%5==0]
        distance=cKDTree(b).query(transform(test,matrix))[0]
        if not ((distance<=.08).mean()>=.85 and np.median(distance)<=.05 and np.linalg.norm(matrix[:3,3])<.5
                and Rotation.from_matrix(matrix[:3,:3]).magnitude()<.26):raise RuntimeError('Measured registration consistency failed')
        registry[entry['id']]=(read(folder/'dataset/frames.json'),matrix)
    counts={key:0 for key in registry}
    for row in rows[len(initial):]:
        original,matrix=registry[row['source_session']];old=original[row['source_frame_index']]
        if not (row['stamp_ns']==old['stamp_ns'] and row['sha256']==old['sha256'] and
                np.allclose(row['c2w'],matrix@np.array(old['c2w']),atol=1e-10)):
            raise RuntimeError('Supplemental pose/image origin mismatch')
        if sha(output/'dataset'/row['file_path'])!=old['sha256']:raise RuntimeError('Supplemental image bytes changed')
        counts[row['source_session']]+=1
    return dict(verified=True,primary_native_map_unchanged=True,supplemental_images=counts,
        no_truth_registration=True,scope=manifest['scope'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['create','add','verify'])
    p.add_argument('output',type=Path);p.add_argument('--source',type=Path)
    a=p.parse_args()
    if a.mode=='create':create(a.source,a.output)
    elif a.mode=='add':add(a.source,a.output)
    else:print(json.dumps(verify_bundle(a.output),indent=2))
