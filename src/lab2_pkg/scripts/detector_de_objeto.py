#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

class BlueSquareDetector:
    def __init__(self):
        rospy.init_node('blue_square_position')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        self.blue_square_pub = rospy.Publisher('/blue_square_position', Vector3, queue_size=10)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_cb)
        self.depth_image_np = None
        self.cv_image = None
        self.vector = Vector3(0, 0, 0)
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        



    def depth_image_cb(self, msg):
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
                        return self.depth_image_np[i][j] #devuelve la distancia del robot al cuadrado azul



    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV) # Transformar la imagen al espacio de color HSV
        mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue) # Aplicar una máscara para detectar los píxeles azules
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Encontrar los contornos de los objetos detectados

        blue_square_contour = None
        for contour in contours:  # Filtrar los contornos para encontrar el cuadrado azul haciendo uso de la aproximación de polígonos
            perimeter = cv2.arcLength(contour, True) #perímetro del contorno
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True) #aproximación de polígonos con la función approxPolyDP de OpenCV
            if len(approx) == 4:
                blue_square_contour = approx
                break

        if blue_square_contour is not None: # Calcular la posición horizontal x del cuadrado azul con respecto al centro de la imagen
            M = cv2.moments(blue_square_contour) #momento de orden 0 para el área.
            if M['m00'] != 0:  #M['m00'] es el área debido a que el cuadrado azul es un contorno cerrado. y se escribe como m00 en lugar de m0, porque es un momento de orden 0.
                cX = int(M['m10'] / M['m00']) #cX es el centroide del contorno en el eje x.
                image_width = hsv_image.shape[1] #image_width es el ancho de la imagen.
                center_x = image_width // 2 #center_x es el centro de la imagen en el eje x.
                position_x = cX - center_x

            """position_y = self.blue_square_depht(blue_square_contour) # Encontrar la distancia media del cuadrado azul. """
            
            angulo_yaw = np.arctan2(cX, center_x) # la tangente inversa de la división del centroide del contorno en el eje x y el centro de la imagen en el eje x es el angulo yaw del robot con respecto al cuadrado azul.
            
            # Publicar la posición del cuadrado azul
            position_msg = Vector3(position_x, 0, angulo_yaw)
            self.blue_square_pub.publish(position_msg)
            rospy.loginfo("Blue square position: {}".format(position_msg))
            

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('blue_square_detector')
    detector = BlueSquareDetector()
    detector.run()