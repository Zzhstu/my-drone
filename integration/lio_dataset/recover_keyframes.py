"""Recover only actual bag JPEGs, using bracketed original measured LIO states."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from poses import PoseBuffer


def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def candidates(output):
    import rosbag
    g=json.loads((output/'sensor_geometry.json').read_text())
    ext=np.linalg.inv(np.array(g['body_from_imu']))@np.array(g['body_from_camera'])
    states=[json.loads(s) for s in (output/'trajectory_imu.jsonl').read_text().splitlines()]
    poses=PoseBuffer(ext,capacity=len(states)+1)
    for row in states:poses.add(row['stamp_ns'],row['map_from_imu'])
    existing={r['stamp_ns'] for r in json.loads((output/'dataset/frames.json').read_text())};rows=[]
    quality=json.loads((output/'quality_filter.json').read_text()) if (output/'quality_filter.json').exists() else {}
    ambiguous=set(quality.get('ambiguous_measurement_stamps',[]))
    with rosbag.Bag(str(output/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/lio_dataset/rgb/compressed']):
            stamp=msg.header.stamp.to_nsec()
            if stamp in existing or stamp in ambiguous:continue
            solved=poses.camera(stamp)
            if solved is None:continue
            pose,association=solved
            rows.append(dict(stamp_ns=stamp,file_path='images/%019d.jpg'%stamp,c2w=pose.tolist(),pose_association=association,
                sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),source='Actual bag acquisition JPEG; recovered original bracketed FAST-LIO states + model optical extrinsic'))
    write(output/'recovery_candidates.json',rows);print('Actual unused bag JPEG candidates',len(rows),flush=True)


def select(output):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'roomgs'))
    from warehouse20_audit import intersector,view_mask,has_translation_baseline
    data=np.load(output/'mesh_reference.npz');state=np.load(output/'coverage_samples.npz')
    missing=np.flatnonzero(~state['complete']);p,n=[data[key][missing] for key in ('points','normals')]
    ray=intersector(data['triangles']);alignment=np.array(json.loads((output/'audit_alignment.json').read_text())['world_from_estimator'])
    info=json.loads((output/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    original=json.loads((output/'dataset/frames.json').read_text());rows=json.loads((output/'recovery_candidates.json').read_text())
    observed=[[] for _ in missing]
    for row in original:
        pose=alignment@np.array(row['c2w']);seen=view_mask(ray,p,n,pose,k,info['width'],info['height'])
        for j in np.flatnonzero(seen):observed[j].append(pose[:3,3])
    masks=[];centers=[]
    for row in rows:
        pose=alignment@np.array(row['c2w']);masks.append(view_mask(ray,p,n,pose,k,info['width'],info['height']));centers.append(pose[:3,3])
    masks=np.array(masks);centers=np.array(centers);done=np.zeros(len(missing),bool);selected=[];available=np.ones(len(rows),bool)
    for _ in range(50):
        far=np.zeros_like(masks)
        for j,observations in enumerate(observed):
            if observations:far[:,j]=(np.linalg.norm(centers[:,None]-np.array(observations)[None],axis=-1)>=.4-1e-9).any(1)
        score=3*(masks&far&~done).sum(1)+(masks&np.array([not x for x in observed])&~done).sum(1)
        score=score.astype(float);score[~available]=-1;i=int(np.argmax(score))
        if score[i]<=0:break
        selected.append(rows[i]);available[i]=False
        for j in np.flatnonzero(masks[i]):
            observed[j].append(centers[i]);done[j]=has_translation_baseline(np.array(observed[j]))
        if done.all():break
    write(output/'recovery_selection.json',dict(original_missing=len(missing),recovered_support=int(done.sum()),remaining=int((~done).sum()),selected=selected,
        policy='Offline fixed-target selection among actually acquired JPEGs only; no synthesized views, extrapolated poses or changed denominator'))
    print('Selected actual images',len(selected),'recovered supports',int(done.sum()),'remaining',int((~done).sum()),flush=True)


def apply(output):
    import rosbag
    report=json.loads((output/'recovery_selection.json').read_text());selected={r['stamp_ns']:r for r in report['selected']}
    if not selected:print('No extra valid observations available',flush=True);return
    rows=json.loads((output/'dataset/frames.json').read_text());existing={r['stamp_ns'] for r in rows}
    if existing&set(selected):raise ValueError('Recovery already applied')
    for name in ['dataset/frames.json','dataset/frames.jsonl','coverage.json','coverage_samples.npz','result.json']:
        backup=output/(name.replace('/','_')+'.before_recovery')
        if not backup.exists():shutil.copyfile(output/name,backup)
    found=set()
    with rosbag.Bag(str(output/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/lio_dataset/rgb/compressed']):
            stamp=msg.header.stamp.to_nsec()
            if stamp not in selected:continue
            row=selected[stamp];data=bytes(msg.data)
            if hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('Bag image changed')
            path=output/'dataset'/row['file_path']
            if path.exists():raise ValueError('Image already exists')
            path.write_bytes(data);found.add(stamp)
    if found!=set(selected):raise ValueError('Actual selected JPEG missing')
    rows=sorted(rows+list(selected.values()),key=lambda r:r['stamp_ns']);write(output/'dataset/frames.json',rows)
    (output/'dataset/frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    write(output/'recovery_result.json',dict(applied=True,added_images=len(selected),original_images=len(existing),final_images=len(rows),source='Actual same-session bag and original LIO trajectory',selected_sha256={str(k):v['sha256'] for k,v in selected.items()}))
    print('Recovered real JPEG frames',len(selected),'total',len(rows),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['candidates','select','apply']);parser.add_argument('output')
    args=parser.parse_args();globals()[args.mode](Path(args.output))
