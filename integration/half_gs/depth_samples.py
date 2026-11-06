"""Project nearest actual registered LiDAR scan into each measured RGB frame."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import rosbag
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.lio_dataset.poses import xyz_from_cloud


def run(root):
    root=Path(root);frames=json.loads((root/'dataset/frames.json').read_text());camera=json.loads((root/'camera_input.json').read_text())
    k=np.array(camera['K']).reshape(3,3);width,height=camera['width'],camera['height'];destination=root/'dataset/lidar_depth';destination.mkdir(exist_ok=False)
    index=0;previous=None;records=[]
    def export(i,choice):
        row=frames[i];stamp=row['stamp_ns'];difference=abs(choice[0]-stamp) if choice else 10**18
        if difference>80000000:
            records.append(dict(index=i,available=False,nearest_gap_ns=difference));return
        pose=np.array(row['c2w']);p=(xyz_from_cloud(choice[1])-pose[:3,3])@pose[:3,:3];z=p[:,2]
        uv=p[:,:2]/np.maximum(z[:,None],.001);uv=uv*np.array([k[0,0],k[1,1]])+k[:2,2]
        valid=(z>.15)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        pix=uv[valid].astype(np.int32);zz=z[valid].astype(np.float32)
        name=Path(row['file_path']).stem+'.npz';np.savez_compressed(destination/name,uv=pix,z=zz,cloud_stamp_ns=choice[0],rgb_stamp_ns=stamp)
        records.append(dict(index=i,available=True,path=name,cloud_stamp_ns=choice[0],rgb_stamp_ns=stamp,gap_ns=difference,points=len(pix)))
    with rosbag.Bag(str(root/'sensors.bag')) as bag:
        for _,message,_ in bag.read_messages(topics=['/cloud_registered']):
            current=(message.header.stamp.to_nsec(),message)
            while index<len(frames) and frames[index]['stamp_ns']<=current[0]:
                stamp=frames[index]['stamp_ns'];choice=min([a for a in [previous,current] if a is not None],key=lambda a:abs(a[0]-stamp))
                export(index,choice);index+=1
            previous=current
    while index<len(frames):export(index,previous);index+=1
    report=dict(frames=len(frames),available=sum(r['available'] for r in records),maximum_gap_ns=80000000,
        source='Actual /cloud_registered at its measurement stamp, projected using actual RGB LIO optical pose; not a depth camera or simulator rendering',records=records)
    (root/'lidar_depth_manifest.json').write_text(json.dumps(report,indent=2)+'\n');print('Sparse measured depths',report['available'],'/',len(frames),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');run(p.parse_args().dataset)
