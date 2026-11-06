#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import select
import termios
import tty

import rospy
from std_msgs.msg import Bool
from prometheus_msgs.msg import GimbalControl


def make_speed_msg(roll_speed=0.0, pitch_speed=0.0, yaw_speed=0.0):
    msg = GimbalControl()
    msg.mode = 1
    msg.angle = [0.0, 0.0, 0.0]
    msg.speed = [float(roll_speed), float(pitch_speed), float(yaw_speed)]
    return msg


def make_angle_msg(roll=0.0, pitch=0.0, yaw=0.0):
    msg = GimbalControl()
    msg.mode = 2
    msg.angle = [float(roll), float(pitch), float(yaw)]
    msg.speed = [0.0, 0.0, 0.0]
    return msg


def get_key(timeout=0.15):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


def publish_for(pub, msg, duration, hz):
    rate = rospy.Rate(hz)
    end_t = time.time() + duration
    while not rospy.is_shutdown() and time.time() < end_t:
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node('gimbal_manual_teleop', anonymous=True)

    uav_id = int(rospy.get_param('~uav_id', 1))
    control_topic = rospy.get_param('~control_topic', '/uav%d/gimbal/control' % uav_id)
    override_topic = rospy.get_param('~manual_override_topic', '/uav%d/p450/manual_override' % uav_id)

    rate_hz = float(rospy.get_param('~rate_hz', 20.0))
    pulse_time = float(rospy.get_param('~pulse_time', 0.35))

    pitch_speed = float(rospy.get_param('~pitch_speed', 20.0))
    yaw_speed = float(rospy.get_param('~yaw_speed', 25.0))

    center_roll = float(rospy.get_param('~center_roll', 0.0))
    center_pitch = float(rospy.get_param('~center_pitch', 0.0))
    center_yaw = float(rospy.get_param('~center_yaw', 0.0))

    pub = rospy.Publisher(control_topic, GimbalControl, queue_size=1)
    pub_override = rospy.Publisher(override_topic, Bool, queue_size=1)

    def release_manual_override():
        try:
            pub.publish(make_speed_msg(0.0, 0.0, 0.0))
            pub_override.publish(Bool(data=False))
            rospy.logwarn('manual_override=False published to %s', override_topic)
            rospy.sleep(0.2)
        except Exception:
            pass

    rospy.on_shutdown(release_manual_override)
    rospy.sleep(0.5)

    pub_override.publish(Bool(data=True))
    rospy.logwarn('manual_override=True published to %s', override_topic)

    print("""
=== Gimbal Manual Teleop, manual override enabled ===
W/Up    : pitch up by speed pulse
S/Down  : pitch down by speed pulse
A/Left  : yaw left by speed pulse
D/Right : yaw right by speed pulse
R       : recenter by angle command
Space   : stop speed now
Q       : quit and release manual override

Topic:
  control : %s
  override: %s
""" % (control_topic, override_topic))

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    zero = make_speed_msg(0.0, 0.0, 0.0)

    try:
        while not rospy.is_shutdown():
            pub_override.publish(Bool(data=True))

            key = get_key(0.15)
            if key == '':
                continue

            key = key.lower()

            if key == 'q':
                print('Quit teleop')
                publish_for(pub, zero, 0.3, rate_hz)
                break

            elif key == ' ':
                print('STOP speed')
                publish_for(pub, zero, 0.5, rate_hz)

            elif key == 'r':
                print('RECENTER angle roll=%.1f pitch=%.1f yaw=%.1f' %
                      (center_roll, center_pitch, center_yaw))
                publish_for(
                    pub,
                    make_angle_msg(center_roll, center_pitch, center_yaw),
                    1.0,
                    rate_hz
                )
                publish_for(pub, zero, 0.3, rate_hz)

            elif key == 'w':
                print('pitch up speed')
                publish_for(pub, make_speed_msg(0.0, +pitch_speed, 0.0), pulse_time, rate_hz)
                publish_for(pub, zero, 0.15, rate_hz)

            elif key == 's':
                print('pitch down speed')
                publish_for(pub, make_speed_msg(0.0, -pitch_speed, 0.0), pulse_time, rate_hz)
                publish_for(pub, zero, 0.15, rate_hz)

            elif key == 'a':
                print('yaw left speed')
                publish_for(pub, make_speed_msg(0.0, 0.0, +yaw_speed), pulse_time, rate_hz)
                publish_for(pub, zero, 0.15, rate_hz)

            elif key == 'd':
                print('yaw right speed')
                publish_for(pub, make_speed_msg(0.0, 0.0, -yaw_speed), pulse_time, rate_hz)
                publish_for(pub, zero, 0.15, rate_hz)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        release_manual_override()
        print('Quit teleop, manual override released.')


if __name__ == '__main__':
    main()