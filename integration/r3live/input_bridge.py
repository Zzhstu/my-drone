#!/usr/bin/env python3
"""Adapt real measurements to R3LIVE's PointXYZINormal input; never use poses."""
import json
import threading
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField
from std_msgs.msg import String


def convert_cloud(msg, profile, stride=2, blind=.5, maximum=25.):
    fields = {f.name: f for f in msg.fields}
    types = {2: 'u1', 4: 'u2', 6: 'u4', 7: 'f4', 8: 'f8'}
    names = [n for n in ('x', 'y', 'z', 'intensity', 'offset_time') if n in fields]
    if not {'x', 'y', 'z'}.issubset(names):
        raise ValueError('PointCloud2 requires x/y/z')
    if profile != 'sim' and 'offset_time' not in names:
        raise ValueError('Real Livox input requires original offset_time nanoseconds')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype(dict(names=names, formats=[endian+types[fields[n].datatype] for n in names],
                         offsets=[fields[n].offset for n in names], itemsize=msg.point_step))
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step)).reshape(-1)[::stride]
    xyz = np.column_stack([raw[n] for n in ('x', 'y', 'z')])
    r2 = np.sum(xyz*xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & (r2 >= blind*blind) & (r2 <= maximum*maximum)
    xyz, raw = xyz[valid], raw[valid]
    points = np.zeros((len(xyz), 8), dtype='<f4')
    points[:, :3] = xyz
    if 'intensity' in names: points[:, 3] = raw['intensity']
    if 'offset_time' in names:
        points[:, 7] = raw['offset_time'].astype(float) * 1e-6  # ns to ms, upstream convention
        points = points[np.argsort(points[:, 7], kind='stable')]
    # Gazebo's existing XYZ scan is instantaneous: zero offsets are explicit,
    # not invented acquisition times. This mode must not be used for real scans.
    out = PointCloud2(header=msg.header, height=1, width=len(points), is_bigendian=False,
                     is_dense=True, point_step=32, row_step=32*len(points), data=points.tobytes())
    out.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z', 'intensity', 'normal_x', 'normal_y', 'normal_z', 'curvature'))]
    return out


def main():
    rospy.init_node('astra_r3live_input')
    profile = rospy.get_param('~profile', 'sim')
    if profile not in ('sim', 'livox_pointcloud2'): raise ValueError('Unknown input profile')
    bridge = CvBridge()
    cloud_pub = rospy.Publisher('/r3live/input/cloud', PointCloud2, queue_size=1)
    image_pub = rospy.Publisher('/r3live/input/image', Image, queue_size=1)
    image_rate=float(rospy.get_param('~image_rate',10.))
    jpeg_quality=int(rospy.get_param('~jpeg_quality',0))
    if image_rate<=0 or not (jpeg_quality==0 or 1<=jpeg_quality<=100):raise ValueError('Invalid RGB rate/quality')
    compressed_pub=rospy.Publisher('/r3live/input/image/compressed',CompressedImage,queue_size=1) if jpeg_quality else None
    point_stride=int(rospy.get_param('~point_step',2))
    stats_pub = rospy.Publisher('/r3live/input/status', String, queue_size=1, latch=True)
    stats = dict(clouds=0, images=0, points=0, errors=0, profile=profile,image_rate_hz=image_rate,jpeg_quality=jpeg_quality,point_stride=point_stride)
    lock = threading.Lock()
    last_image = [float('-inf')]
    def cloud(msg):
        try:
            out = convert_cloud(msg, profile,point_stride)
            if out.width < 30: return
            cloud_pub.publish(out)
            with lock: stats.update(clouds=stats['clouds']+1, points=stats['points']+out.width)
        except Exception as error:
            with lock: stats['errors'] += 1
            rospy.logerr_throttle(2, str(error))
    def image(msg):
        stamp = msg.header.stamp.to_sec()
        if stamp-last_image[0] < .95/image_rate: return
        try:
            bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if compressed_pub is not None:
                ok,encoded=cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,jpeg_quality])
                if not ok:raise RuntimeError('Acquisition JPEG encoding failed')
                packed=CompressedImage(header=msg.header,format='jpeg',data=encoded.tobytes())
                # R3LIVE sees the decoded same JPEG that is archived/trained.
                bgr=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
                compressed_pub.publish(packed)
            out = bridge.cv2_to_imgmsg(bgr, encoding='bgr8'); out.header = msg.header
            image_pub.publish(out); last_image[0] = stamp
            with lock: stats['images'] += 1
        except Exception as error:
            with lock: stats['errors'] += 1
            rospy.logerr_throttle(2, str(error))
    rospy.Subscriber(rospy.get_param('~cloud_topic', '/livox/lidar'), PointCloud2, cloud, queue_size=3, buff_size=8000000)
    rospy.Subscriber(rospy.get_param('~image_topic'), Image, image, queue_size=3, buff_size=8000000)
    def status(_):
        with lock: stats_pub.publish(String(data=json.dumps(stats)))
    rospy.Timer(rospy.Duration(1), status)
    rospy.spin()


if __name__ == '__main__': main()
