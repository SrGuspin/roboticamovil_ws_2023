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

        while self.lin_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.angular_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.lineal_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        self.ang_set_point.publish(0)
        self.lin_set_point.publish(0)

        self.mover_sub = rospy.Subscriber(
            '/blue_square_position', Vector3(), self.accion_mover_cb)

        rospy.sleep(0.2)

    def velocidad_angular(self, data):
        self.angular_speed = float(data.data)

    def aplicar_velocidad(self, displacement_list):
        # displacement_list = [obj_aux_yaw_1,obj_x, obj_aux_yaw_2, obj_y, obj_ang]
        for i, displacement in enumerate(displacement_list):
            condicional = None
            if i == 1:
                self.lineal_speed = 0.2
                condicional = True
            else:
                self.ang_set_point.publish(displacement[1])
                condicional = False
            rospy.sleep(0.5)
            speed = Twist()  # clase a la cual mandar los datos
            contador = 0
            while True:
                if condicional is None:
                    break
                elif not condicional:  # condicional solo dice si es lineal o angular
                    speed.linear.x = 0  # solo cuando está en angular
                    if round(self.angular_speed, 3) == 0:
                        contador += 1
                        pass
                else:
                    speed.linear.x = self.lineal_speed
                    if abs(self.x - displacement[0]) <= 0.2:
                        break
                if contador >= 25:
                    print("break")
                    break
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
        rospy.loginfo([round(self.x, 3), round(self.y, 3), round(self.yaw, 3)])
        if self.yaw < 0:
            self.yaw += 2*np.pi
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
        pos_blue_square_x = vector.x
        angulo_blue_square = vector.z
        distancia_obj = self.distancia_blue_square(pos_blue_square_x)
        angulo_obj = self.yaw + angulo_blue_square
        if x == 0:
            self.position_x = True
        if not self.position_x:
            rospy.loginfo((0, 0, angulo_obj))
            rospy.sleep(2)
            self.aplicar_velocidad([angulo_obj])
        else:
            self.aplicar_velocidad([angulo_obj, distancia_obj])


if __name__ == '__main__':

    mic = SeguidorDeObjeto()
    mic.x = 0
    mic.y = 0
    mic.yaw = 0
    mic.vector = Vector3(0, 0, 0)
    rospy.spin()

# https://stackoverflow.com/questions/35942754/how-can-i-save-username-and-password-in-git
