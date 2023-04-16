#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class Turtlebot_Kinect(object):
    def __init__(self):
        self.depth_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_cb)
        self.rgb_sub = rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.rgb_cb)
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.current_cv_rgb_image = None

    def depth_cb(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)

    def rgb_cb(self, data):
        self.current_cv_rgb_image = self.bridge.imgmsg_to_cv2(data)


if __name__ == '__main__':
    rospy.init_node('test_kinect')
    handler = Turtlebot_Kinect()
    rospy.spin()
