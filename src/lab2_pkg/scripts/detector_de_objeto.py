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
        self.image_sub = rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.rgb_image_callback)
        self.blue_square_pub = rospy.Publisher(
            '/blue_square_position', Vector3, queue_size=10)
        self.depth_image_np = None
        self.cv_image = None
        self.vector = Vector3(0, 0, 0)
        self.lower_blue = np.array([50, 100, 20])
        self.upper_blue = np.array([125, 255, 255])

        ######### CAMARA DE PROFUNDIDAD #########
    """ def depth_image_cb(self, msg):
        try:
            self.depth_image_np = self.bridge.imgmsg_to_cv2(msg)
            self.vector = self.blue_square_depht()
        except CvBridgeError as e:
            rospy.logerr(e)

    def blue_square_depht(self, blue_square_contour):
        if self.depth_image_np is not None: #compara la imagen de profundidad con la del cuadrado azul para definir la distancia del robot al cuadrado. hazlo comparando los pixeles de la imagen de profundidad con los pixeles de la imagen del cuadrado azul y viendo si los del cuadrado azul tienen la misma profundida, para poder diferenciar de algun otro objeto azul que pueda haber en la imagen. Hazlo con un for que recorra la imagen de profundidad y que compare los pixeles de la imagen de profundidad con los pixeles de la imagen del cuadrado azul y viendo si los del cuadrado azul tienen la misma profundida, para poder diferenciar de algun otro objeto azul que pueda haber en la imagen
            for i in range(self.depth_image_np.shape[0]):
                for j in range(self.depth_image_np.shape[1]):
                    if self.depth_image_np[i][j] == blue_square_contour[i][j]:
                        return self.depth_image_np[i][j] #devuelve la distancia del robot al cuadrado azul """

    def rgb_image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return e

        # Transformar la imagen al espacio de color HSV
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2HSV)
        # Aplicar una máscara para detectar los píxeles azules
        mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)
        # Encontrar los contornos de los objetos detectados
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blue_square_contour = None
        area = 500
        approx = None
        for contour in contours:
            if area < cv2.contourArea(contour):
                perimeter = cv2.arcLength(contour, True)
                # aproximación de polígonos con la función approxPolyDP de OpenCV
                approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                area = cv2.contourArea(contour)
            if approx is not None:
                if len(approx) == 4:
                    blue_square_contour = approx
                    break

        position_x = 0
        angulo_yaw = 0
        distancia = 0
        position_msg = [position_x, distancia, angulo_yaw]
        center_x = 0
        cX = 0

        if blue_square_contour is not None:  # Calcular la posición horizontal x del cuadrado azul con respecto al centro de la imagen
            # momento de orden 0 para el área.
            M = cv2.moments(blue_square_contour)
            # M['m00'] es el área debido a que el cuadrado azul es un contorno cerrado. y se escribe como m00 en lugar de m0, porque es un momento de orden 0.
            if M['m00'] != 0 and area > 1000:
                # cX es el centroide del contorno en el eje x.
                cX = int(M['m10'] / M['m00'])
                # image_width es el ancho de la imagen.
                image_width = hsv_image.shape[1]
                # center_x es el centro de la imagen en el eje x.
                center_x = image_width // 2
                position_x = cX - center_x

                """ ##################   DISTANCIA   ##################
                if (2 * (cX - center_x) * math.tan(0.523599)) != 0: # si sabemos que el lado del cuadrado son 15 cm, entonces podemos calcular la distancia del robot al cuadrado azul, la camara tiene 57° de angulo de vision horizontal:
                    distancia = (14 * 640) / (2 * (cX - center_x) * math.tan(0.523599))
                else:
                    distancia = 0 """

                ##################   ANGULO   ##################
            # angulo_yaw es el ángulo de orientación del robot con respecto al cuadrado azul.
            angulo_yaw = np.arctan2(cX, center_x)

            # Publicar la posición del cuadrado azul
            position_msg = Vector3(position_x, distancia, angulo_yaw)
            self.blue_square_pub.publish(position_msg)
            rospy.loginfo("Blue square position: {}".format(position_msg))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    detector = BlueSquareDetector()
    while not rospy.is_shutdown():
        rospy.spin()
