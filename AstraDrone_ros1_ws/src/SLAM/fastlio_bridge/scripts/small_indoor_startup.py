#!/usr/bin/env python3
"""Office-only FCU heading readiness before fixed map/ENU initialization."""
import math
import time


class HeadingWindow:
    """Require fresh, distinct stamped attitudes stable over a dwell interval."""
    def __init__(self, seconds=3., tolerance_deg=2.):
        self.seconds=seconds
        self.tolerance=math.radians(tolerance_deg)
        self.samples=[]

    def update(self, stamp, quaternion, ready=True):
        q=tuple(float(v) for v in quaternion)
        if not ready or not math.isfinite(stamp) or len(q)!=4 or not all(math.isfinite(v) for v in q):
            self.samples=[]
            return False
        norm=math.sqrt(sum(v*v for v in q))
        if norm<.5:
            self.samples=[]
            return False
        q=tuple(v/norm for v in q)
        if self.samples and stamp<=self.samples[-1][0]:
            if stamp<self.samples[-1][0]:
                self.samples=[]
            return False
        self.samples.append((stamp,q))
        # A 90-degree EKF reset must start a completely new stable interval.
        anchor=self.samples[0][1]
        angle=2*math.acos(min(1.,abs(sum(a*b for a,b in zip(anchor,q)))))
        if angle>self.tolerance:
            self.samples=[(stamp,q)]
        return stamp-self.samples[0][0]>=self.seconds


def wait_heading_ready(process, timeout=90.):
    """Do not lock the bridge to the early PX4 yaw-initialization transient.

    Position-only ground readiness can occur before the simulator's first
    yaw alignment. Require connected/disarmed state, fresh attitude, a startup settling
    interval and then a distinct-stamp stable heading window. Uses FCU data,
    never Gazebo truth or room geometry. Does not guarantee future reset-free
    flight; the office velocity adapter handles the map/local yaw transform.
    """
    import threading
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    latest={};lock=threading.Lock();window=HeadingWindow()
    def receive(key,msg):
        with lock:
            latest[key]=(msg,time.monotonic())
    subscribers=[rospy.Subscriber('/mavros/local_position/pose',PoseStamped,lambda m:receive('pose',m),queue_size=1),
                 rospy.Subscriber('/mavros/state',State,lambda m:receive('state',m),queue_size=1)]
    start_sim=rospy.Time.now().to_sec();deadline=time.monotonic()+timeout
    try:
        while time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError('Gazebo/PX4 exited while waiting for heading')
            now=rospy.Time.now().to_sec()
            with lock:
                pose,state=latest.get('pose'),latest.get('state')
            ready=bool(pose and state and state[0].connected and not state[0].armed and
                       now-start_sim>=12. and
                       time.monotonic()-pose[1]<1. and time.monotonic()-state[1]<4. and
                       0<=now-pose[0].header.stamp.to_sec()<=.5)
            if pose:
                q=pose[0].pose.orientation
                if window.update(pose[0].header.stamp.to_sec(),(q.x,q.y,q.z,q.w),ready):
                    return pose[0]
            time.sleep(.05)
    finally:
        for sub in subscribers:
            sub.unregister()
    raise RuntimeError('PX4 connected/disarmed / stable heading not ready within wall timeout')
