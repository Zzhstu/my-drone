#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class WaypointsToMarker:
    def __init__(self):
        self.waypoints_param = rospy.get_param('~waypoints_param', '/book_qr_patrol_mission/waypoints')
        self.marker_topic = rospy.get_param('~marker_topic', '/book_qr_waypoints_markers')
        self.path_topic = rospy.get_param('~planned_path_topic', '/book_qr_waypoints_path')
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.line_width = float(rospy.get_param('~line_width', 0.05))
        self.point_scale = float(rospy.get_param('~point_scale', 0.12))
        self.text_scale = float(rospy.get_param('~text_scale', 0.20))
        self.z_lift = float(rospy.get_param('~z_lift', 0.0))

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)

        rospy.sleep(0.5)
        self.publish_once()
        rospy.loginfo('waypoints_to_marker published: frame_id=%s waypoints_param=%s marker_topic=%s planned_path_topic=%s',
                      self.frame_id, self.waypoints_param, self.marker_topic, self.path_topic)

    def _mk_color(self, r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def publish_once(self):
        items = rospy.get_param(self.waypoints_param, [])
        if not items:
            rospy.logwarn('waypoints_to_marker: no waypoints found at %s', self.waypoints_param)
            return

        marker_array = MarkerArray()
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()

        # 1) total route line strip
        route = Marker()
        route.header.frame_id = self.frame_id
        route.header.stamp = rospy.Time.now()
        route.ns = 'book_qr_route'
        route.id = 0
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = self.line_width
        route.color = self._mk_color(0.1, 0.8, 1.0, 1.0)

        # 2) scan-only route line strip
        scan_route = Marker()
        scan_route.header.frame_id = self.frame_id
        scan_route.header.stamp = rospy.Time.now()
        scan_route.ns = 'book_qr_scan_route'
        scan_route.id = 1
        scan_route.type = Marker.LINE_STRIP
        scan_route.action = Marker.ADD
        scan_route.pose.orientation.w = 1.0
        scan_route.scale.x = self.line_width * 0.8
        scan_route.color = self._mk_color(1.0, 0.6, 0.1, 1.0)

        marker_array.markers.extend([route, scan_route])

        marker_id = 10
        for idx, item in enumerate(items):
            x = float(item.get('x', 0.0))
            y = float(item.get('y', 0.0))
            z = float(item.get('z', 0.0)) + self.z_lift
            name = str(item.get('name', f'wp_{idx:02d}'))
            stage = str(item.get('stage', ''))
            scan = bool(item.get('scan', True))

            p = Point(x=x, y=y, z=z)
            route.points.append(p)
            if scan:
                scan_route.points.append(p)

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = p
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'book_qr_waypoints'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = p
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = self.point_scale
            sphere.scale.y = self.point_scale
            sphere.scale.z = self.point_scale
            sphere.color = self._mk_color(0.1, 0.9, 0.1, 1.0) if scan else self._mk_color(0.9, 0.9, 0.1, 1.0)
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = 'book_qr_waypoint_labels'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = z + max(0.18, self.point_scale * 1.2)
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale
            text.color = self._mk_color(1.0, 1.0, 1.0, 1.0)
            text.text = f'{idx:02d} {name} ({stage})'
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)
        self.path_pub.publish(path)


if __name__ == '__main__':
    rospy.init_node('waypoints_to_marker', anonymous=False)
    WaypointsToMarker()
    rospy.spin()
