#!/usr/bin/env python3
"""Global loop-map coordinates for planning; continuous local map for control.

Only an atomic LoopMapSnapshot is accepted. This module never modifies the
FAST-LIO state, MAVROS origin, or a recorded/local message in place.
"""
import bisect
import copy
import json
import math
import numpy as np


def quaternion(q):
    a=np.array([q.x,q.y,q.z,q.w],float)
    if not np.isfinite(a).all() or np.linalg.norm(a)<1e-8:raise ValueError('invalid quaternion')
    return a/np.linalg.norm(a)


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def matrix(pose):
    m=np.eye(4);m[:3,:3]=rotation(quaternion(pose.orientation))
    m[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
    if not np.isfinite(m).all():raise ValueError('nonfinite pose')
    return m


def from_transform(message):
    m=np.eye(4);t=message.transform
    m[:3,:3]=rotation(quaternion(t.rotation));m[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    if not np.isfinite(m).all():raise ValueError('nonfinite map transform')
    return m


def multiply(a,b):
    av,aw=a[:3],a[3];bv,bw=b[:3],b[3]
    return np.r_[aw*bv+bw*av+np.cross(av,bv),aw*bw-np.dot(av,bv)]


def quat_from_rotation(m):
    # Global corrections preserve gravity and therefore rotate about Z.
    # Explicitly reject unsupported roll/pitch instead of silently dropping it.
    if not np.allclose(m[2,:],[0,0,1],atol=1e-6) or not np.allclose(m[:,2],[0,0,1],atol=1e-6):
        raise ValueError('online planner requires gravity-aligned loop correction')
    angle=math.atan2(m[1,0],m[0,0]);return np.array([0.,0.,math.sin(angle/2),math.cos(angle/2)])


def transform_pose(pose,m):
    result=copy.deepcopy(pose);p=m[:3,:3]@np.array([pose.position.x,pose.position.y,pose.position.z])+m[:3,3]
    result.position.x,result.position.y,result.position.z=map(float,p)
    q=multiply(quat_from_rotation(m[:3,:3]),quaternion(pose.orientation))
    q/=np.linalg.norm(q)
    result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=map(float,q)
    return result


def transform_message(message,m,kind):
    if message.header.frame_id.lstrip('/')!='map':raise ValueError('expected continuous local map input')
    result=copy.copy(message);result.header=copy.deepcopy(message.header);result.header.frame_id='loop_map'
    if np.array_equal(m,np.eye(4)):return result
    if kind=='odom':
        result.pose=copy.deepcopy(message.pose);result.pose.pose=transform_pose(message.pose.pose,m)
    elif kind=='scan':
        fields={f.name:f for f in message.fields};payload=bytearray(message.data);views=[]
        for name in ('x','y','z'):
            f=fields.get(name)
            if f is None or f.datatype!=7:raise ValueError('expected FLOAT32 XYZ')
            views.append(np.ndarray((message.height,message.width),dtype=('>' if message.is_bigendian else '<')+'f4',
                buffer=payload,offset=f.offset,strides=(message.row_step,message.point_step)))
        p=np.stack([v.ravel() for v in views],axis=1);good=np.isfinite(p).all(axis=1)
        p[good]=p[good]@m[:3,:3].T+m[:3,3]
        for i,v in enumerate(views):v[:]=p[:,i].reshape(v.shape)
        result.data=bytes(payload)
    else:raise ValueError('unsupported planning input '+kind)
    return result


class PlanningSnapshot:
    def __init__(self,message):
        stamp=message.header.stamp.to_sec()
        if message.header.frame_id!='loop_map' or not math.isfinite(stamp) or stamp<=0:
            raise ValueError('invalid global snapshot header')
        for cloud in (message.map,message.free,message.suspects):
            if cloud.header.frame_id!='loop_map' or cloud.header.stamp!=message.header.stamp:
                raise ValueError('mixed map/ghost/free coordinate version or acquisition time')
        t=message.global_from_local
        if t.header.frame_id!='loop_map' or t.child_frame_id!='map' or t.header.stamp!=message.header.stamp:
            raise ValueError('snapshot transform does not match map')
        self.transform=from_transform(t);quat_from_rotation(self.transform[:3,:3])
        self.inverse=np.linalg.inv(self.transform)
        self.history=json.loads(message.ghost_history.data);self.status=json.loads(message.ghost_status.data)
        if (self.history.get('version')!=1 or self.history.get('frame_id')!='loop_map' or
            abs(self.history.get('map_stamp',-1)-stamp)>1e-6 or not isinstance(self.history.get('records'),list)):
            raise ValueError('ghost ledger is not paired with global map')
        self.message=message;self.stamps=[];self.corrections=[]
        if message.graph_revision:
            g=message.graph
            if (g.revision!=message.graph_revision or g.raw.header.frame_id!='map' or
                g.optimized.header.frame_id!='loop_map' or not g.raw.poses or len(g.raw.poses)!=len(g.optimized.poses)):
                raise ValueError('unpaired graph snapshot')
            for raw,opt in zip(g.raw.poses,g.optimized.poses):
                t=raw.header.stamp.to_sec()
                if raw.header.stamp!=opt.header.stamp or self.stamps and t<=self.stamps[-1]:
                    raise ValueError('graph stamp mismatch')
                self.stamps.append(t);self.corrections.append(matrix(opt.pose)@np.linalg.inv(matrix(raw.pose)))

    def at(self,stamp):
        if not self.stamps:return np.eye(4)
        i=bisect.bisect_right(self.stamps,stamp)
        if not i:return self.corrections[0]
        if i==len(self.stamps):return self.corrections[-1]
        w=(stamp-self.stamps[i-1])/(self.stamps[i]-self.stamps[i-1])
        a,b=self.corrections[i-1],self.corrections[i]
        yaw_a,yaw_b=math.atan2(a[1,0],a[0,0]),math.atan2(b[1,0],b[0,0])
        yaw=yaw_a+w*math.atan2(math.sin(yaw_b-yaw_a),math.cos(yaw_b-yaw_a))
        m=np.eye(4);c,s=math.cos(yaw),math.sin(yaw);m[:2,:2]=[[c,-s],[s,c]]
        m[:3,3]=(1-w)*a[:3,3]+w*b[:3,3];return m
