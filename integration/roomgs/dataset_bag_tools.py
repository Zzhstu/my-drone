"""Recover exact native RGB/optical pairs already recorded in an actual bag.

ROS Python entry. This never interpolates poses or reads simulator geometry.
Use only after recorder/collector have stopped. Original dataset is backed up.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import rosbag
from geometry import matrix_from_pose


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def poses(output):
    output=Path(output);rows=[]
    with rosbag.Bag(str(output/'sensors.bag')) as bag:
        for _,message,_ in bag.read_messages(topics=['/roomgs/optical_pose']):
            rows.append(dict(stamp_ns=message.header.stamp.to_nsec(),c2w=matrix_from_pose(message.pose).tolist(),
                source='actual native successful R3LIVE optical pose, original measurement timestamp'))
    path=output/'dataset/native_optical_poses.json'
    path.write_text(json.dumps(rows,indent=2)+'\n')
    print('Exported native measured poses',len(rows),flush=True)


def images(output,stamps):
    import rospy
    import cv2
    from cv_bridge import CvBridge
    output=Path(output);folder=output/'dataset'
    selected=json.loads(Path(stamps).read_text());wanted=set(selected['additional_stamp_ns'])
    original=folder/'frames.json'
    if (output/'sensors.bag.active').exists():raise RuntimeError('Recorder still active')
    rows=json.loads(original.read_text());old={r['stamp_ns'] for r in rows}
    wanted-=old
    native={r['stamp_ns']:r for r in json.loads((folder/'native_optical_poses.json').read_text())}
    if any(stamp not in native for stamp in wanted):raise RuntimeError('Requested pose missing from actual bag export')
    backup=folder/'before_bag_extension';backup.mkdir(exist_ok=False)
    for name in ('frames.json','frames.jsonl','export_status.json'):
        if (folder/name).exists():shutil.copyfile(folder/name,backup/name)
    initial_sha=sha(original);added=[];bridge=CvBridge()
    with rosbag.Bag(str(output/'sensors.bag')) as bag:
        compressed='/r3live/input/image/compressed' in bag.get_type_and_topic_info()[1]
        topic='/r3live/input/image/compressed' if compressed else '/r3live/input/image'
        for _,message,_ in bag.read_messages(topics=[topic]):
            stamp=message.header.stamp.to_nsec()
            if stamp not in wanted:continue
            path=folder/'images'/(('%019d.jpg' if compressed else '%019d.png')%stamp)
            if path.exists():raise RuntimeError('Unindexed image already exists; do not overwrite '+str(path))
            if compressed:
                if message.format!='jpeg':raise RuntimeError('Unexpected compressed acquisition format')
                path.write_bytes(bytes(message.data))
            elif not cv2.imwrite(str(path),bridge.imgmsg_to_cv2(message,'bgr8')):raise RuntimeError('Image save failed')
            row=dict(file_path='images/'+path.name,stamp_ns=stamp,c2w=native[stamp]['c2w'],sha256=sha(path),
                source='actual successful native optical pose and exact original input RGB timestamp recovered from bag')
            added.append(row);wanted.remove(stamp)
            if not wanted:break
    if wanted:raise RuntimeError('Exact input RGB unavailable for requested actual poses; original frame index remains unchanged')
    rows=sorted(rows+added,key=lambda r:r['stamp_ns'])
    original.write_text(json.dumps(rows,indent=2)+'\n')
    (folder/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    report=dict(original_frames_sha256=initial_sha,new_frames_sha256=sha(original),
        original_exported=len(old),additional_exported=len(added),new_exported=len(rows),
        additional_stamp_ns=[r['stamp_ns'] for r in added],selection_sha256=sha(stamps),
        native_pose_export_sha256=sha(folder/'native_optical_poses.json'),
        source_sha256=sha(__file__),map_sha256=sha(output/'rgb_pt.pcd'),
        scope='Only additional real exact native image/pose pairs; no changes to map/reference/coverage criteria',
        accepted_complete=False)
    (output/'dataset_extension.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['poses','images']);parser.add_argument('capture');parser.add_argument('--stamps')
    args=parser.parse_args()
    if args.mode=='poses':poses(args.capture)
    else:images(args.capture,args.stamps)
