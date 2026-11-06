"""Pose and native PCD I/O shared by export/audit; no simulator dependencies."""
import numpy as np


def matrix_from_pose(pose):
    p,q=pose.position,pose.orientation
    q=np.array([q.x,q.y,q.z,q.w],dtype=float);q/=np.linalg.norm(q)
    x,y,z,w=q
    t=np.eye(4)
    t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    t[:3,3]=[p.x,p.y,p.z]
    return t


def read_pcd(path):
    with open(path,'rb') as f:
        h={}
        while True:
            line=f.readline()
            if not line:raise ValueError('Incomplete PCD header')
            line=line.decode('ascii').strip()
            if not line or line.startswith('#'):continue
            key,*values=line.split();h[key]=values
            if key=='DATA':break
        if h['DATA']!=['binary']:raise ValueError('Expected native binary PCD')
        types={'F':'f','I':'i','U':'u'}
        dtype=np.dtype([(n,'<'+types[t]+s) for n,t,s in zip(h['FIELDS'],h['TYPE'],h['SIZE'])])
        points=np.frombuffer(f.read(),dtype=dtype,count=int(h['POINTS'][0]))
    xyz=np.column_stack([points[n] for n in ('x','y','z')])
    color=points['rgb'].view('<u4')
    rgb=np.column_stack([(color>>16)&255,(color>>8)&255,color&255]).astype(np.uint8)
    valid=np.isfinite(xyz).all(axis=1)
    return xyz[valid],rgb[valid]


def transform(points,t):return points@t[:3,:3].T+t[:3,3]
