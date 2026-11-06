#!/usr/bin/env python3
"""Low-flow actual sensor adapter. No image localization or depth camera."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CompressedImage,PointCloud2
from integration.r3live.input_bridge import convert_cloud


def main():
    rospy.init_node('lio_dataset_sensors');bridge=CvBridge()
    cloud=rospy.Publisher('/lio_dataset/cloud',PointCloud2,queue_size=1)
    rgb=rospy.Publisher('/lio_dataset/rgb/compressed',CompressedImage,queue_size=1)
    def scan(msg):cloud.publish(convert_cloud(msg,'sim',stride=6))
    def image(msg):
        ok,data=cv2.imencode('.jpg',bridge.imgmsg_to_cv2(msg,'bgr8'),[cv2.IMWRITE_JPEG_QUALITY,92])
        if not ok:raise RuntimeError('JPEG encoding failed')
        rgb.publish(CompressedImage(header=msg.header,format='jpeg',data=data.tobytes()))
    rospy.Subscriber('/livox/lidar',PointCloud2,scan,queue_size=1,buff_size=8000000)
    rospy.Subscriber(rospy.get_param('~image_topic'),Image,image,queue_size=1,buff_size=2000000)
    rospy.spin()


if __name__=='__main__':main()
