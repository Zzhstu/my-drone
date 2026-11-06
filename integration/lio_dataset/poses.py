"""Measurement-time camera poses; no ROS, camera tracking or simulator truth."""
from bisect import bisect_left
from collections import deque
import numpy as np


def quaternion(matrix):
    """Rotation matrix to unit quaternion, x y z w; NumPy-only for ROS Python."""
    r=np.asarray(matrix);q=np.empty(4);trace=np.trace(r)
    if trace>0:
        s=2*np.sqrt(trace+1);q[:]=[(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,s/4]
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=2*np.sqrt(1+r[i,i]-r[j,j]-r[k,k]);q[i]=s/4
        q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
    return q/np.linalg.norm(q)


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def slerp(a,b,alpha):
    a,b=quaternion(a),quaternion(b);dot=float(np.dot(a,b))
    if dot<0:b=-b;dot=-dot
    if dot>.9995:q=(1-alpha)*a+alpha*b
    else:
        angle=np.arccos(np.clip(dot,-1,1))
        q=(np.sin((1-alpha)*angle)*a+np.sin(alpha*angle)*b)/np.sin(angle)
    return rotation(q)


class PoseBuffer:
    def __init__(self, imu_from_camera, capacity=200, max_gap_ns=150000000):
        self.extrinsic=np.asarray(imu_from_camera,dtype=float)
        self.stamps=deque(maxlen=capacity);self.poses=deque(maxlen=capacity)
        self.max_gap_ns=max_gap_ns

    def add(self, stamp, pose):
        if self.stamps and stamp<=self.stamps[-1]:
            raise ValueError('LIO timestamps must increase strictly')
        pose=np.asarray(pose,dtype=float)
        if not np.isfinite(pose).all() or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-5):
            raise ValueError('Invalid LIO transform')
        self.stamps.append(int(stamp));self.poses.append(pose.copy())

    def camera(self, stamp):
        if not self.stamps or stamp<self.stamps[0] or stamp>self.stamps[-1]:return None
        index=bisect_left(self.stamps,stamp)
        if self.stamps[index]==stamp:
            return self.poses[index]@self.extrinsic,dict(left_stamp_ns=stamp,right_stamp_ns=stamp,alpha=0.,method='exact_LIO_state')
        left,right=self.stamps[index-1],self.stamps[index]
        if right-left>self.max_gap_ns:return None
        alpha=(stamp-left)/(right-left);a,b=self.poses[index-1],self.poses[index]
        pose=np.eye(4);pose[:3,3]=a[:3,3]*(1-alpha)+b[:3,3]*alpha
        pose[:3,:3]=slerp(a[:3,:3],b[:3,:3],alpha)
        return pose@self.extrinsic,dict(left_stamp_ns=left,right_stamp_ns=right,alpha=float(alpha),method='bracketed_LIO_linear_translation_SO3_SLERP')


def xyz_from_cloud(msg):
    fields={f.name:f for f in msg.fields};endian='>' if msg.is_bigendian else '<'
    dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
        offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
    rows=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([rows[k] for k in ('x','y','z')])
    return xyz[np.isfinite(xyz).all(1)]


class VoxelMap:
    """One measured point per voxel; bounded memory, no CAD surface filling."""
    def __init__(self, size=.05, maximum=1000000):
        self.size=size;self.maximum=maximum;self.points={}

    def add(self, xyz):
        keys=np.floor(xyz/self.size).astype(np.int32)
        _,ids=np.unique(keys,axis=0,return_index=True)
        for i in ids:
            key=tuple(keys[i])
            if key not in self.points:
                if len(self.points)>=self.maximum:raise RuntimeError('Measured map point budget exceeded')
                self.points[key]=xyz[i].copy()

    def array(self):return np.asarray(list(self.points.values()),dtype=np.float32).reshape(-1,3)
