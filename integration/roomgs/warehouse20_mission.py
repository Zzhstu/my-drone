"""Known-envelope inspection, two sides with overhead transfers; no truth feeds."""
import math
import numpy as np

SPAWN=np.array([-1.3,-1.2,.06])


def waypoints():
    route=[]
    def add(x,y,z,yaw):route.append([x-SPAWN[0],y-SPAWN[1],z-SPAWN[2],yaw])
    add(-1.3,-1.2,1.,math.pi/2)
    # All low transfers stay on their respective side. The 3.6m rack leaves
    # only 0.69m at its ends, so every side change goes above the rack.
    for side in [-1,1]:
        add(-1.3,side*1.2,2.35,-side*math.pi/2)
        for j,z in enumerate([.48,.85,1.25,1.65,2.0,2.35]):
            xs=np.linspace(-1.9,1.9,9)
            if j%2:xs=xs[::-1]
            for x in xs:add(float(x),side*1.25,z,-side*math.pi/2)
        # Wall-facing strips include low/high viewpoints for floor/ceiling.
        for j,z in enumerate([2.35,1.25,.48]):
            xs=np.linspace(-1.9,1.9,9)
            if j%2:xs=xs[::-1]
            for x in xs:add(float(x),side*.9,z,side*math.pi/2)
        # Corner panoramas cover the short walls and rack ends without trying
        # to put the airframe into an end gap narrower than its rotor span.
        for z in [.48,1.25,2.35]:
            for x in [-1.9,1.9]:
                for yaw in [0,math.pi/2,math.pi,-math.pi/2]:
                    add(x,side*1.25,z,yaw)
        add(-1.3,side*1.2,2.35,-side*math.pi/2)
    add(-1.3,-1.2,2.35,math.pi/2)
    add(-1.3,-1.2,1.,math.pi/2)
    return route


def ideal_camera_poses(body_from_camera,spacing=.15):
    poses=[];route=waypoints()
    for a,b in zip(route[:-1],route[1:]):
        a=np.array(a);b=np.array(b);delta=(b[3]-a[3]+math.pi)%(2*math.pi)-math.pi
        count=max(2,int(np.ceil(np.linalg.norm(b[:3]-a[:3])/spacing)),int(abs(delta)/.15))
        for u in np.linspace(0,1,count,endpoint=False):
            angle=a[3]+delta*u;c,s=math.cos(angle),math.sin(angle)
            body=np.eye(4);body[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
            body[:3,3]=a[:3]*(1-u)+b[:3]*u+SPAWN
            poses.append(body@body_from_camera)
    return poses
