#!/usr/bin/env python3
"""Validate original measurements and export common known-pose dataset layouts."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from poses import PoseBuffer,quaternion


def dump(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def export(output):
    output=Path(output);dataset=output/'dataset'
    info=json.loads((output/'camera_input.json').read_text())
    if any(abs(d)>1e-8 for d in info['D']):raise ValueError('Undistortion required before PINHOLE export')
    g=json.loads((output/'sensor_geometry.json').read_text())
    extrinsic=np.linalg.inv(np.array(g['body_from_imu']))@np.array(g['body_from_camera'])
    trajectory=[json.loads(s) for s in (output/'trajectory_imu.jsonl').read_text().splitlines()]
    poses=PoseBuffer(extrinsic,capacity=max(2,len(trajectory)))
    for row in trajectory:poses.add(row['stamp_ns'],row['map_from_imu'])
    rows=json.loads((dataset/'frames.json').read_text());previous=-1;max_error=0.;max_gap=0
    for row in rows:
        stamp=row['stamp_ns'];path=dataset/row['file_path']
        if stamp<=previous or path.stem!='%019d'%stamp:raise ValueError('Invalid measurement timestamp/order')
        if sha(path)!=row['sha256']:raise ValueError('Image checksum changed')
        image=cv2.imread(str(path))
        if image is None or image.shape[:2]!=(info['height'],info['width']):raise ValueError('Image decode/dimensions mismatch')
        derived=poses.camera(stamp)
        if derived is None:raise ValueError('Image lacks valid measured LIO brackets')
        camera,association=derived
        if row['pose_association']!=association:raise ValueError('Pose association metadata changed')
        error=float(np.max(np.abs(camera-np.array(row['c2w']))));max_error=max(max_error,error)
        if error>1e-9:raise ValueError('Camera pose does not match measured LIO + SDF extrinsic')
        previous=stamp;max_gap=max(max_gap,association['right_stamp_ns']-association['left_stamp_ns'])
    xyz=np.load(output/'static_map.npz')['xyz']
    if len(rows)<21 or len(xyz)<100 or not np.isfinite(xyz).all():raise ValueError('Insufficient/invalid dataset')
    k=np.array(info['K']).reshape(3,3);axis=np.diag([1.,-1.,-1.,1.])
    transforms=dict(camera_model='OPENCV',fl_x=float(k[0,0]),fl_y=float(k[1,1]),cx=float(k[0,2]),cy=float(k[1,2]),
        w=info['width'],h=info['height'],k1=0.,k2=0.,p1=0.,p2=0.,
        coordinate_convention='OpenGL camera x right, y up, z backward; world remains FAST-LIO camera_init',
        frames=[dict(file_path=row['file_path'],transform_matrix=(np.array(row['c2w'])@axis).tolist(),stamp_ns=row['stamp_ns']) for row in rows])
    dump(dataset/'transforms.json',transforms)
    sparse=dataset/'sparse/0';sparse.mkdir(parents=True,exist_ok=True)
    (sparse/'cameras.txt').write_text('# Known calibration, no SfM estimation\n1 PINHOLE %d %d %.15g %.15g %.15g %.15g\n'%(info['width'],info['height'],k[0,0],k[1,1],k[0,2],k[1,2]))
    with (sparse/'images.txt').open('w') as f,(dataset/'poses.csv').open('w') as csv_file:
        f.write('# Known optical poses. Empty POINTS2D lines: no feature tracks reconstructed.\n')
        writer=csv.writer(csv_file);writer.writerow(['stamp_ns','file_path','tx','ty','tz','qx','qy','qz','qw'])
        for i,row in enumerate(rows,1):
            c2w=np.array(row['c2w']);w2c=np.linalg.inv(c2w);q=quaternion(w2c[:3,:3])[[3,0,1,2]]
            values=[*q,*w2c[:3,3]]
            f.write('%d %s 1 %s\n\n'%(i,' '.join('%.17g'%v for v in values),Path(row['file_path']).name))
            writer.writerow([row['stamp_ns'],row['file_path'],*c2w[:3,3],*quaternion(c2w[:3,:3])])
    with (sparse/'points3D.txt').open('w') as f:
        f.write('# Measured LiDAR XYZ. RGB=128 is explicitly UNKNOWN neutral initialization, not measured color. No SfM tracks.\n')
        for i,p in enumerate(xyz,1):f.write('%d %.9g %.9g %.9g 128 128 128 0\n'%(i,*p))
    dump(output/'coordinate_conventions.json',dict(map_frame='camera_init',units='metres',
        lio_odometry_child='FAST-LIO /Odometry child_frame_id=body actually denotes the IMU state, not airframe origin',
        native_frames='dataset/frames.json: c2w maps optical camera x-right/y-down/z-forward into map',
        formula='T_map_optical(t_rgb)=T_map_imu(t_rgb) @ inverse(T_body_imu) @ T_body_optical',
        intrinsics='camera_input.json, PINHOLE with zero distortion in this simulation',
        nerf='dataset/transforms.json uses OpenGL optical axes: T_map_opengl=T_map_optical @ diag(1,-1,-1,1)',
        colmap='dataset/sparse/0 text: world-to-camera, quaternion qw qx qy qz, PINHOLE, no feature tracks; conversion to .bin may be required by chosen trainer',
        point_colors='static_map.ply/pcd/npz contain XYZ only; COLMAP RGB=128 denotes unknown neutral initialization',
        truth='No simulator truth/CAD in exported camera poses, measured cloud, calibration-derived transforms or COLMAP/NeRF exports',
        interpolation='Bracketed linear translation + SO3 SLERP of actual 10Hz LIO states, max 150ms; not direct per-image visual estimates; no extrapolation'))
    report=dict(passed=True,images=len(rows),map_points=len(xyz),lio_states=len(trajectory),
        max_recomputed_c2w_abs_error=max_error,max_lio_bracket_gap_ms=max_gap/1e6,
        duration_seconds=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9,
        image_sha256_verified=True,all_images_decoded=True,calibration_chain_recomputed=True,
        input_sha256={name:sha(output/name) for name in ['static_map.npz','camera_input.json','sensor_geometry.json','trajectory_imu.jsonl','dataset/frames.json']},
        scope='Data integrity and measured transform chain only; not pixel accuracy, whole-room coverage or 3DGS quality')
    dump(output/'dataset_validation.json',report);print(json.dumps(report),flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output');export(parser.parse_args().output)
