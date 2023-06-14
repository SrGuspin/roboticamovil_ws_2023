#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math


def getAngle(a, b, c):
    p1 = [a[0]-b[0], a[1]-b[1]]
    p2 = [c[0]-b[0], c[1]-b[1]]
    ang = math.atan2(p2[1], p2[0]) - math.atan2(p1[1], p1[0])

    if ang >= math.pi:
        ang -= 2*math.pi
    elif ang < -math.pi:
        ang += 2*math.pi

    return ang


class SeguidorDeObjeto(object):

    def __init__(self):
        rospy.init_node('seguidor_de_objeto')
        self.position_x = False
        self.ref_lin = 0
        self.ref_ang = 0
        self.lineal_speed = 0
        self.angular_speed = 0
        self.pos = 0
        self.eje = False
        self.cmd_vel_mux_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)

        # Lectura odometria
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_cb)

        # ESTADOS PID
        self.ang_set_point = rospy.Publisher(
            '/robot_angular/setpoint', Float64, queue_size=1)

        self.angular_state = rospy.Publisher(
            '/robot_angular/state', Float64, queue_size=1)

        self.ang_actuation = rospy.Subscriber(
            '/robot_angular/control_effort', Float64, self.velocidad_angular)

        while self.angular_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        self.ang_set_point.publish(0)

        self.mover_sub = rospy.Subscriber(
            '/blue_square_position', Vector3, self.accion_mover_cb)

        rospy.sleep(0.2)

    def velocidad_angular(self, data):
        self.angular_speed = float(data.data)

    def aplicar_velocidad(self, angulo):

        speed = Twist()  # clase a la cual mandar los datos
        speed.linear.x = self.lineal_speed
        speed.angular.z = self.angular_speed  # correciones mientras se mueve lineal
        # manda la velocidad al kubuki
        self.cmd_vel_mux_pub.publish(speed)
        self.rate_obj.sleep()

    def odometry_cb(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        roll, pitch, self.yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                       odom.pose.pose.orientation.y,
                                                       odom.pose.pose.orientation.z,
                                                       odom.pose.pose.orientation.w))
        # rospy.loginfo([round(self.x, 3), round(self.y, 3), round(self.yaw, 3)])
        self.angular_state.publish(self.yaw)  # donde está

    def distancia_blue_square(self, pos_blue_square_x):
        # si sabemos que el lado del cuadrado son 15 cm, entonces podemos calcular la distancia del robot al cuadrado azul, la camara tiene 57° de angulo de vision horizontal:
        if (2 * (pos_blue_square_x) * math.tan(0.523599)) != 0:
            distancia = (14 * 640) / \
                (2 * (pos_blue_square_x) * math.tan(0.523599))
        else:
            distancia = 0
        return distancia

    def accion_mover_cb(self, vector):
        # recibe unicamente el punto en donde esta el cubo azul.
        # Como Vector3 (x, y, z) el valor de z se ignora
        # topico importante /goal_list

        pos_x = vector.x
        ancho = vector.y
        mitad = ancho / 2
        campo_visual = np.deg2rad(28.5)

        angulo = ((campo_visual*pos_x)/mitad) - campo_visual
        error = (-1 * angulo) + self.yaw

        cosa = np.rad2deg(angulo)

        if vector.z == 1:
            self.lineal_speed = 0
            angulo = self.yaw
        else:
            self.lineal_speed = 0.2

        if not np.isnan(cosa):
            self.ang_set_point.publish(error)
            rospy.loginfo(f"angulo = {cosa}")
            rospy.loginfo(f"error = {np.rad2deg(error)}")
            rospy.loginfo(f"yaw = {np.rad2deg(self.yaw)}")

            self.aplicar_velocidad(angulo)
            rospy.sleep(1/100)


if __name__ == '__main__':

    mic = SeguidorDeObjeto()
    mic.x = 0
    mic.y = 0
    mic.yaw = 0
    mic.vector = Vector3(0, 0, 0)
    rospy.spin()

# https://stackoverflow.com/questions/35942754/how-can-i-save-username-and-password-in-git
