##! /usr/bin/env python3
# Hasta aqui funciona la llegada pero es reactivo en su movimiento

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Twist
from tf.transformations import euler_from_quaternion

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
        self.vector = Vector3(0, 0, 0)
        self.depth_image_np = None
        self.resolucion = 0.01
        self.velocidad_lineal = 0.2
        self.i = 0
        self.giro = 0

        self.ref_center = 0
        self.angular_speed = 0
        self.pos = 0
        self.eje = False
        self.fin = False

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
        # rospy.init_node('turtlebot')

        self.depth_img_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_image_cb)
        self.rgb_img_sub = rospy.Subscriber(
            '/camera/rgb/image_color', Image, self.rgb_image_cb)
        self.cmd_vel_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.occupancy_state_pub = rospy.Publisher(
            '/occupancy_state', Vector3, queue_size=10)

        while self.center_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.center_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        self.center_set_point.publish(0)

        # self.mover_sub = rospy.Subscriber(
        #    '/goal_list', PoseArray, self.accion_mover_cb)

        rospy.sleep(0.2)
        self.aplicar_velocidad()

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        roll, pitch, self.yaw = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                       msg.pose.pose.orientation.y,
                                                       msg.pose.pose.orientation.z,
                                                       msg.pose.pose.orientation.w))
        rospy.loginfo('Current pose - lin: (%f, %f) ang: (%f)' %
                      (self.x, self.y, self.yaw))

        if self.yaw < 0:
            self.yaw = self.yaw + 2*np.pi
        self.center_state.publish(self.yaw)

    def depth_image_cb(self, msg):
        try:
            self.depth_image_np = self.bridge.imgmsg_to_cv2(msg)
            self.vector = self.obtacle_detected()
        except CvBridgeError as e:
            rospy.logerr(e)

    def rgb_image_cb(self, msg):
        try:
            self.rgb_image_np = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def obtacle_detected(self):
        # Es un vector de tres cosas, que incluye la condicion, idea:
        # (0, 0, 0) -> sigue derecho
        # (1, 0, 0) -> rota a la derecha
        # (0, 1, 0) -> Retrocede
        # (0, 0, 1) -> Rota a la izquierda
        # Los otros casos de por ejemplo, retroceder y girar a la vez,
        # Provocaran que  termine chocando al darse la vuelta, ya que,
        # no tenemos sensores en la parte trasera del equipo

        obstacle = False
        if self.depth_image_np is not None:
            columna1 = self.depth_image_np[:, 0]
            columna2 = self.depth_image_np[:, 320]
            columna3 = self.depth_image_np[:, 639]

            columna1 = np.where(np.isnan(columna1), 0.0, columna1)
            columna2 = np.where(np.isnan(columna2), 0.0, columna2)
            columna3 = np.where(np.isnan(columna3), 0.0, columna3)

            obstacle1 = np.any(columna1 < 0.5)
            obstacle2 = np.any(columna2 < 0.5)
            obstacle3 = np.any(columna3 < 0.5)
            columnas = Vector3(columna1, columna2, columna3)
            self.centro_tunel = (np.mean(columna3) -
                                 np.mean(columna1))/2  # - 1.25
            self.pos_vs_centro = self.y - self.centro_tunel
            rospy.loginfo('columna1: %f' % np.mean(columna1))
            rospy.loginfo('columna2: %f' % np.mean(columna3))
            rospy.loginfo('centro tunel: %f' % self.centro_tunel)
            rospy.loginfo('diferencia a referencia: %f' % self.pos_vs_centro)

            frente = (self.x+np.cos(self.yaw), self.y+np.sin(self.yaw))
            # angulo = getAngle(frente, (self.x, self.y),
            #                   (self.x, self.centro_tunel))
            angulo = getAngle(frente, (self.x, self.y),
                              (self.x, self.pos_vs_centro))
            obj_aux_yaw_1 = self.yaw + angulo  # MANDAR A PID

            # columna 1 = izquierda
            # columna 2 = frente
            # columna 3 = derecha
            # rospy.loginfo("columna1: {}".format(np.mean(columna1)))

            if obstacle1:
                obstacle1 = 1
            else:
                obstacle1 = 0

            if obstacle2:
                obstacle2 = 1
            else:
                obstacle2 = 0

            if obstacle3:
                obstacle3 = 1
            else:
                obstacle3 = 0
            self.center_set_point.publish(obj_aux_yaw_1)
            # self.center_state.publish(abs(self.y-1.25))
            vector = Vector3(obstacle1, obstacle2, obstacle3)
            self.occupancy_state_pub.publish(vector)
        else:
            vector = Vector3(0, 0, 0)
        return vector

    def aplicar_velocidad(self):
        speed = Twist()

        while not rospy.is_shutdown():  # CONDICIONES
            speed.linear.x = self.velocidad_lineal
            speed.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(speed)
            self.run()
            self.rate_obj.sleep()

    def run(self):
        free_space = True
        giro_der = -0.35
        giro_izq = 0.35
        vector = self.vector

        rospy.loginfo([vector.x, vector.y, vector.z])

        if (vector.x == 1 and vector.z == 1) or self.fin:
            if self.i < 60:
                self.i += 1
                self.velocidad_lineal = 0
                self.fin = True
                self.angular_speed = self.giro
            else:
                while True:
                    break

        elif vector.x == 1:
            self.velocidad_lineal = 0.2
        elif vector.z == 1:
            self.velocidad_lineal = 0.2
        else:
            self.velocidad_lineal = 0.2

    def velocidad_angular(self, data):
        if not self.fin:
            self.angular_speed = float(data.data)
        else:
            self.angular_speed = 0
            self.giro = self.detectar_imagen()

    def detectar_imagen(self):
        # img = cv2.imread(cv2.samples.findFile(
        #     '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/scripts/arrow_left.png'))
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
            giro = -0.35
        else:
            giro = 0.35
        return giro


if __name__ == '__main__':

    rospy.init_node('reactive_movement')
    turtlebot = TurtlebotController()
    turtlebot.run()
    rospy.spin()
