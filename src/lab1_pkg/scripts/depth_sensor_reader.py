#! /usr/bin/env python3

import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ReadDepthSensor(object):

    def __init__(self):
        rospy.init_node('read_depth_sender')
        self.bridge = CvBridge()
        self.pub_depth_sensor = rospy.Publisher(
            '/depth_sensor/img', Image, queue_size=10)
        self.depth_img_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_image_cb)
        self.depth_image_np = None

    def depth_image_cb(self, msg):

        try:
            self.depth_image_np = self.bridge.imgmsg_to_cv2(msg)

            img2display = self.depth_image_np - self.depth_image_np.min()
            img2display = (
                img2display * (255.0/img2display.max())).astype(np.uint8)
            img2display = 255 - img2display
            img2display = cv2.applyColorMap(img2display, cv2.COLORMAP_HOT)
            img2display = cv2.flip(img2display, 0)

            img_msg = self.bridge.cv2_to_imgmsg(img2display, "bgr8")

            self.pub_depth_sensor.publish(img_msg)

        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':

    read_depth_sensor = ReadDepthSensor()
    rospy.spin()
