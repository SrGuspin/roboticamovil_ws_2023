#! /usr/bin/env python3
# Hasta aqui funciona la llegada pero es reactivo en su movimiento

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Twist
from tf.transformations import euler_from_quaternion
import shapely as shp
from shapely.geometry import LineString, Point

# from turtlebot_audio import TurtlebotAudio

from std_msgs.msg import String
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError


def getAngle(a, b, c):
    p1 = [a[0]-b[0], a[1]-b[1]]
    p2 = [c[0]-b[0], c[1]-b[1]]
    ang = math.atan2(p2[1], p2[0]) - math.atan2(p1[1], p1[0])

    if ang >= math.pi:
        ang -= 2*math.pi
    elif ang < -math.pi:
        ang += 2*math.pi

    return ang


class TurtlebotController(object):

    def __init__(self):

        self.bridge = CvBridge()
        self.depth_image_np = None

        self.velocidad_lineal = 0.1
        self.angular_speed = 0
        self.fin = False
        self.giro = 0

        self.rate_hz = 10
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        # ESTADOS PID
        self.center_set_point = rospy.Publisher(
            '/robot_angular/setpoint', Float64, queue_size=1)

        self.center_state = rospy.Publisher(
            '/robot_angular/state', Float64, queue_size=1)

        self.ang_actuation = rospy.Subscriber(
            '/robot_angular/control_effort', Float64, self.velocidad_angular)

        self.rate_obj = rospy.Rate(self.rate_hz)

        self.depth_img_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_image_cb)
        self.rgb_img_sub = rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.rgb_image_cb)
        self.cmd_vel_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        while self.center_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.center_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        rospy.sleep(0.2)
        self.period = 1/3
        rospy.Timer(rospy.Duration(self.period), self.obtacle_detected)
        self.aplicar_velocidad()

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        roll, pitch, self.yaw = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                       msg.pose.pose.orientation.y,
                                                       msg.pose.pose.orientation.z,
                                                       msg.pose.pose.orientation.w))
        # rospy.loginfo([self.x, self.y, self.yaw])

        self.center_state.publish(self.yaw)

    def depth_image_cb(self, msg):
        try:
            self.depth_image_np = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def rgb_image_cb(self, msg):
        try:
            self.rgb_image_np = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def obtacle_detected(self, data):

        if self.depth_image_np is not None:
            columna1 = self.depth_image_np[0:213]
            columna2 = self.depth_image_np[213:426]
            columna3 = self.depth_image_np[426:640]

            columna1 = np.where(np.isnan(columna1), 0.0, columna1)
            columna2 = np.where(np.isnan(columna2), 0.0, columna2)
            columna3 = np.where(np.isnan(columna3), 0.0, columna3)

            obstacle1 = np.mean(columna1)
            obstacle2 = np.mean(columna2)
            obstacle3 = np.mean(columna3)

            columnas = Vector3(obstacle1, obstacle2, obstacle3)
            rospy.loginfo([obstacle1, obstacle2, obstacle3])

            # 27 grados por seccion
            # cambiar, creo que deberian de ser 17
            grad18 = np.deg2rad(27)
            obj = self.yaw
            if not self.fin:
                # parametro a cambiar = dist: define la distancia hasta el frente que activa
                # la deteccion de flecha.
                # el and esta para cuando no detecte nada a los lados, cuando sea nan.

                dist = 0.8
                if obstacle2 < dist or (obstacle1 == 0.0 and obstacle2 == 0.0):
                    self.velocidad_lineal = 0
                    giro = self.detectar_imagen()
                    obj = np.deg2rad(giro) + self.yaw
                    rospy.logerr(obj)
                    self.fin = True
                elif obstacle1 < obstacle3:
                    obj = self.yaw - grad18
                elif obstacle3 < obstacle1:
                    obj = self.yaw + grad18
            self.center_set_point.publish(obj)

    def aplicar_velocidad(self):
        speed = Twist()

        while not rospy.is_shutdown():
            speed.linear.x = self.velocidad_lineal
            speed.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(speed)
            self.rate_obj.sleep()

    def velocidad_angular(self, data):
        self.angular_speed = float(data.data)

    def detectar_imagen(self):
        img = self.rgb_image_np
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100,
                                minLineLength=5, maxLineGap=190)
        sumax = 0
        sumay = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            sumax += x1 + x2
            sumay += y1 + y2

        if sumax > 12000:
            rospy.loginfo("Flecha izquierda")
            giro = -90
        else:
            rospy.loginfo("Flecha derecha")
            giro = 90
        return giro

    def detectar_flecha(self):
        # hecho por gus
        # no funciona
        # a veces detecta izquierda cuand otiene que ser derecha
        # lo mismo pasa con el codigo de la pauli pero este requiere mucho más procesamiento que
        # el de la pauli :p
        img = self.rgb_image_np
        rgba_color = np.uint8([[[144, 29, 46]]])
        hsv_color = cv2.cvtColor(rgba_color, cv2.COLOR_RGB2HSV)

        lower_blue = np.array([hsv_color[0][0][0] - 10, 100, 100])
        upper_blue = np.array([hsv_color[0][0][0] + 10, 255, 255])

        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Aplicar una máscara para detectar los píxeles azules
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        img = cv2.bitwise_and(img, img, mask=mask)
        img = cv2.medianBlur(img, 5)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(img, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/360, 60)

        extremo = None
        inicio = None
        punta = None
        horizontal = []
        diag = []
        for line in lines:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            if x2 - x1 == 0:
                pendiente = 0
            elif y2 - y1 == 0:
                pendiente = 0
            else:
                pendiente = round((y2-y1)/(x2-x1))
            if pendiente == 0:
                horizontal.append(((x1, y1), (x2, y2)))
            else:
                diag.append(((x1, y1), (x2, y2)))

        diag_sorted = sorted(diag, key=lambda x: x[1][1])

        linea_1 = LineString(diag_sorted[0])
        linea_2 = LineString(diag_sorted[-1])

        interseccion = linea_1.intersection(linea_2)

        mitad = img.shape[1]/2
        x = interseccion.x
        if mitad < x:
            rospy.loginfo("IZQUIERDA")
            return 90
        else:
            rospy.loginfo("DERECHA")
            return -90


if __name__ == '__main__':

    rospy.init_node('reactive_movement')
    turtlebot = TurtlebotController()
    rospy.spin()
