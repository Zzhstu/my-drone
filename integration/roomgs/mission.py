"""Predetermined inspection envelope, not an unknown-room exploration planner.

No scene mesh, audit targets, simulator poses or completeness feedback is read.
The envelope is a user mission specification, in metres from the takeoff frame.
"""
import math
import numpy as np


def waypoints():
    result=[]
    def add(x,y,z,offset=0.):
        # Nominal takeoff is (-3,-2) relative to this inspection envelope.
        result.append([x+3,y+2,z,math.atan2(-y,-x)+offset])
    def ring(z,offset=0.):
        corners=[(-4.3,-5.1),(4.3,-5.1),(4.3,5.1),(-4.3,5.1),(-4.3,-5.1)]
        for a,b in zip(corners[:-1],corners[1:]):
            count=int(math.ceil(math.dist(a,b)/1.3))
            for t in np.linspace(0,1,count,endpoint=False):
                x,y=np.array(a)*(1-t)+np.array(b)*t;add(float(x),float(y),z,offset)
        add(*corners[-1],z,offset)
    add(-3,-2,.95)
    ring(.55,.30)
    add(-2,-2,1.65)
    for a in np.linspace(-math.pi*3/4,math.pi*5/4,25):
        add(1.6*math.cos(a),1.6*math.sin(a),1.65)
    ring(1.95,-.30)
    add(-2,-2,2.85)
    ring(2.85)
    # Dedicated tangential and wall-facing inspection strips. A camera which
    # always looks at the room centre misses low surfaces behind furniture.
    def strip(a,b,z,angle):
        count=max(2,int(math.ceil(math.dist(a,b)/.9)))
        for u in np.linspace(0,1,count+1):
            x,y=np.array(a)*(1-u)+np.array(b)*u
            result.append([float(x)+3,float(y)+2,z,angle])
    strip((-4.3,5.1),(4.3,5.1),.55,0.)
    strip((4.3,5.1),(-4.3,5.1),1.4,math.pi)
    strip((-4.3,5.1),(4.3,5.1),2.2,-math.pi/2)
    strip((-4.3,5.1),(-4.3,1.8),.55,.40)
    strip((-4.3,1.8),(-4.3,5.1),1.4,0.)
    strip((-4.3,5.1),(-4.3,1.8),1.4,0.)
    # Return through the clear perimeter rather than cutting across furniture.
    strip((-4.3,1.8),(-4.3,5.1),2.35,0.)
    strip((-4.3,5.1),(4.3,5.1),2.35,0.)
    strip((4.3,5.1),(4.3,-5.1),.65,0.)
    strip((4.3,-5.1),(4.3,5.1),1.45,0.)
    strip((4.3,5.1),(4.3,-5.1),2.35,math.pi)
    # A high return leg clears the existing interior furniture.
    add(-3,-2,2.85)
    add(-3,-2,.95)
    result.append([0.,0.,.95,0.])
    return result


def ideal_camera_poses(body_from_camera,spacing=.45):
    """Offline preflight geometry diagnostic only; never recorded training data."""
    route=waypoints();poses=[]
    for a,b in zip(route[:-1],route[1:]):
        a=np.array(a);b=np.array(b)
        delta=(b[3]-a[3]+math.pi)%(2*math.pi)-math.pi
        count=max(2,int(math.ceil(np.linalg.norm(b[:3]-a[:3])/spacing)))
        for u in np.linspace(0,1,count,endpoint=False):
            xyz=a[:3]*(1-u)+b[:3]*u;angle=a[3]+delta*u
            t=np.eye(4);c,s=math.cos(angle),math.sin(angle)
            t[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
            t[:3,3]=xyz+np.array([-3.,-2.,0.])
            poses.append(t@body_from_camera)
    return poses
