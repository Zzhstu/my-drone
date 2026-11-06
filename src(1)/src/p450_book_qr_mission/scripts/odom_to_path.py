#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath:
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.path_topic = rospy.get_param('~path_topic', '/book_qr_path')
        self.min_dist = float(rospy.get_param('~min_dist', 0.03))
        self.max_len = int(rospy.get_param('~max_len', 6000))
        self.frame_id_override = str(rospy.get_param('~frame_id_override', ''))
        self.publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 10.0))

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10, latch=True)
        self.path_msg = Path()
        self.last_xyz = None
        self.last_pub_t = 0.0

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=100)
        rospy.loginfo('odom_to_path started: odom_topic=%s path_topic=%s min_dist=%.3f max_len=%d frame_override=%s',
                      self.odom_topic, self.path_topic, self.min_dist, self.max_len,
                      self.frame_id_override if self.frame_id_override else '<none>')

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        xyz = (float(pos.x), float(pos.y), float(pos.z))

        if self.last_xyz is not None:
            dx = xyz[0] - self.last_xyz[0]
            dy = xyz[1] - self.last_xyz[1]
            dz = xyz[2] - self.last_xyz[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.min_dist:
                return

        self.last_xyz = xyz

        pose = PoseStamped()
        pose.header = msg.header
        if self.frame_id_override:
            pose.header.frame_id = self.frame_id_override
        pose.pose = msg.pose.pose

        self.path_msg.header = pose.header
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_len:
            self.path_msg.poses = self.path_msg.poses[-self.max_len:]

        now = rospy.get_time()
        if now - self.last_pub_t >= 1.0 / max(self.publish_rate_hz, 1e-3):
            self.last_pub_t = now
            self.path_pub.publish(self.path_msg)


if __name__ == '__main__':
    rospy.init_node('odom_to_path', anonymous=False)
    OdomToPath()
    rospy.spin()
