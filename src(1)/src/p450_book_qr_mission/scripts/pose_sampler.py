#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from prometheus_msgs.msg import UAVState

state = None


def cb(msg):
    global state
    state = msg


def main():
    rospy.init_node('pose_sampler', anonymous=True)
    uav_id = int(rospy.get_param('~uav_id', 1))
    name = rospy.get_param('~name', 'wp_name')
    stage = rospy.get_param('~stage', 'front')
    scan = bool(rospy.get_param('~scan', True))
    hold_sec = float(rospy.get_param('~hold_sec', 1.5))
    scan_timeout_sec = float(rospy.get_param('~scan_timeout_sec', 6.0))
    note = rospy.get_param('~note', '')

    rospy.Subscriber(f'/uav{uav_id}/prometheus/state', UAVState, cb, queue_size=10)

    rospy.loginfo('等待 /uav%d/prometheus/state ...', uav_id)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and state is None:
        rate.sleep()

    if rospy.is_shutdown():
        return

    yaw_deg = math.degrees(float(state.attitude[2]))
    line = (
        f"- {{name: {name}, stage: {stage}, scan: {str(scan).lower()}, "
        f"x: {float(state.position[0]):.3f}, y: {float(state.position[1]):.3f}, z: {float(state.position[2]):.3f}, "
        f"yaw_deg: {yaw_deg:.2f}, hold_sec: {hold_sec:.1f}, scan_timeout_sec: {scan_timeout_sec:.1f}, note: '{note}'}}"
    )
    print('\n' + '=' * 80)
    print('把下面这一行复制到你的航点 yaml 的 waypoints: 下方')
    print(line)
    print('=' * 80 + '\n')


if __name__ == '__main__':
    main()
