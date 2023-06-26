#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
import cv2
import numpy as np
import math


class BlueSquareDetector:
    def __init__(self):
        rospy.init_node('detector_de_objeto')
        self.bridge = CvBridge()
        self.puente2 = CvBridge()
        self.image_sub = rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.rgb_image_callback)
        self.blue_square_pub = rospy.Publisher(
            '/blue_square_position', Vector3, queue_size=1)
        self.depth_image_np = None
        self.cv_image = None
        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.vector = Vector3(0, 0, 0)
        rgba_color = np.uint8([[[83, 134, 217]]])
        hsv_color = cv2.cvtColor(rgba_color, cv2.COLOR_RGB2HSV)

        # Definir el rango de color para filtrar
        self.lower_blue = np.array([hsv_color[0][0][0] - 10, 100, 100])
        self.upper_blue = np.array([hsv_color[0][0][0] + 10, 255, 255])

    def rgb_image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return e

        # Transformar la imagen al espacio de color HSV
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        # Aplicar una máscara para detectar los píxeles azules
        mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)
        img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, 5)
        img = cv2.bitwise_and(img, img, mask=mask)

        # Encontrar los contornos de los objetos detectados
        contours, _ = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blue_square_contour = None
        area = 500
        approx = None
        fig = None

        for contour in contours:
            if cv2.contourArea(contour) > area:
                fig = contour
        if fig is not None:
            M = cv2.moments(fig)
        else:
            M = cv2.moments(mask)

        distancia = 0
        image_width = hsv_image.shape[1]
        center_x = image_width // 2

        if fig is not None and M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # center_x es el centro de la imagen en el eje x.
            # rospy.loginfo(cX)
            position_msg = Vector3(cX, image_width, 0)
            self.blue_square_pub.publish(position_msg)
        else:
            position_msg = Vector3(0, 0, 1)
            self.blue_square_pub.publish(position_msg)
        self.rate_obj.sleep()


if __name__ == '__main__':
    detector = BlueSquareDetector()
    while not rospy.is_shutdown():
        rospy.spin()
