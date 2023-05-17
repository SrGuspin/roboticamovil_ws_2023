#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

class BlueSquareDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.blue_square_pub = rospy.Publisher('blue_square_position', Vector3, queue_size=10)
        # Definir el tipo de mensaje apropiado para publicar la posición del cuadrado azul
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Transformar la imagen al espacio de color HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Aplicar una máscara para detectar los píxeles azules
        mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)

        # Encontrar los contornos de los objetos detectados
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar los contornos para encontrar el cuadrado azul
        blue_square_contour = None
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) == 4:
                blue_square_contour = approx
                break

        if blue_square_contour is not None:
            # Calcular la posición horizontal x del cuadrado azul con respecto al centro de la imagen
            M = cv2.moments(blue_square_contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                image_width = hsv_image.shape[1]
                center_x = image_width // 2
                position_x = cX - center_x

            # Publicar la posición del cuadrado azul
            # Dependiendo del tipo de mensaje que hayas elegido para la posición, aquí debes publicarlo.
            # Por ejemplo, si es un mensaje de tipo Point, podrías hacer algo como esto:
            position_msg = Vector3(position_x, 0, 0)
            self.blue_square_pub.publish(position_msg)
            rospy.loginfo("Blue square position: {}".format(position_msg))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('blue_square_detector')
    detector = BlueSquareDetector()
    rospy.spin()
