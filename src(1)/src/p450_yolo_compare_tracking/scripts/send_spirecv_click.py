#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from spirecv_msgs.msg import Control

if __name__ == '__main__':
    rospy.init_node('send_spirecv_click', anonymous=True)
    uav_id = int(rospy.get_param('~uav_id', 1))
    x = float(rospy.get_param('~x', 0.5))
    y = float(rospy.get_param('~y', 0.5))
    mouse = rospy.get_param('~mouse', 'left')
    topic = rospy.get_param('~topic', '/uav%d/spirecv/control' % uav_id)
    msg = Control()
    msg.mouse = Control.MOUSE_LEFT if mouse.lower() in ['left', 'l', '0'] else Control.MOUSE_RIGHT
    msg.x = x
    msg.y = y
    pub = rospy.Publisher(topic, Control, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(msg)
    rospy.loginfo('published %s click to %s: x=%.3f y=%.3f', mouse, topic, x, y)
    rospy.sleep(0.5)
