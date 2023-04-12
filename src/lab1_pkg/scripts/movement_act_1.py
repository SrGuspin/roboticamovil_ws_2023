#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math


def getAngle(a, b, c):
    p1 = [a[0]-b[0], a[1]-b[1]]
    p2 = [c[0]-b[0], c[1]-b[1]]
    ang = math.atan2(p2[1], p2[0]) - math.atan2(p1[1], p1[0])

    if ang > math.pi:
        ang -= 2*math.pi
    elif ang <= -math.pi:
        ang += 2*math.pi

    # arreglar -1, incluir condicioones
    # cuando es negativo hay que dejarlo negativo???
    return ang * -1


class Movement(object):

    def __init__(self):
        self.max_w = 1.0  # [rad/s]
        self.max_v = 0.2  # [m/s]
        rospy.init_node('dead_reckoning_nav')
        self.cmd_vel_mux_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)

        # Lectura odometria
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_cb)

        self.mover_sub = rospy.Subscriber(
            '/goal_list', PoseArray, self.accion_mover_cb)

    # Funcion del nivel 1.

    def aplicar_velocidad(self, tripleta):
        lin_speed = tripleta[0]
        ang_speed = tripleta[1]
        tiempo = tripleta[2]

        ciclos = int(tiempo*self.rate_hz)
        speed = Twist()
        speed.linear.x = lin_speed
        speed.angular.z = ang_speed
        while not rospy.is_shutdown():
            if ciclos >= 0:
                rospy.loginfo('publishing speed (%f, %f)' %
                              (lin_speed, ang_speed))
                self.cmd_vel_mux_pub.publish(speed)
                self.rate_obj.sleep()
            else:
                break
            ciclos -= 1

    def odometry_cb(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.roll, self.pitch, self.yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                                 odom.pose.pose.orientation.y,
                                                                 odom.pose.pose.orientation.z,
                                                                 odom.pose.pose.orientation.w))
        self.frente = [np.cos(self.yaw)*0.1+self.x,
                       np.sin(self.yaw)*0.1+self.y]
        rospy.logerr([round(self.x, 3), round(self.y, 3), round(self.yaw, 3)])

    # Funcion del nivel 2

    def mover_robot_a_destino(self, goal_pose):
        # angulo de giro
        x = goal_pose[0]
        y = goal_pose[1]
        ang = getAngle([x, y], [self.x, self.y], self.frente)
        # por cuanto giramos
        tiempo_giro = abs(ang/0.1)
        # llamamos a la fuuncion de moverse en linea recta (con 0 en velocidad angular)

        direccion = ang - self.yaw
        if direccion > 0:
            self.aplicar_velocidad([0, 0.1, tiempo_giro])
        else:
            self.aplicar_velocidad([0, -0.1, tiempo_giro])

        # Minimizamos el error
        error = ang - abs(self.yaw)
        tiempo_giro = abs(error/0.05)
        self.aplicar_velocidad([0, 0.05, tiempo_giro])

        punto_1 = np.array([self.x, self.y])
        punto_2 = np.array([goal_pose[0], goal_pose[1]])
        dist_puntos = np.linalg.norm(punto_1 - punto_2)

        tiempo_lin = dist_puntos/0.2
        self.aplicar_velocidad([0.2, 0, tiempo_lin])

        goal_ang = goal_pose[2] - self.yaw
        tiempo_giro = abs(goal_ang/0.1)
        direccion = ang - self.yaw
        if direccion > 0:
            self.aplicar_velocidad([0, 0.1, tiempo_giro])
        else:
            self.aplicar_velocidad([0, -0.1, tiempo_giro])

        error = goal_pose[2] - self.yaw
        tiempo_giro = abs(error/0.05)
        if error > 0:
            self.aplicar_velocidad([0, 0.05, tiempo_giro])
        else:
            self.aplicar_velocidad([0, -0.05, tiempo_giro])

    # Funcion del nivel 3
    def accion_mover_cb(self, algo):
        x = algo.pose.pose.position.x
        y = algo.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                  odom.pose.pose.orientation.y,
                                                  odom.pose.pose.orientation.z,
                                                  odom.pose.pose.orientation.w))
        self.mover_robot_a_destino((x, y, yaw))


if __name__ == '__main__':

    mic = Movement()
    mic.x = 0
    mic.y = 0
    mic.yaw = 0
    mic.frente = [0.1, 0]
    mic.mover_robot_a_destino((1, 0, 0))
    rospy.logerr("Lugar Alcanzado!")
    mic.mover_robot_a_destino((1, 1, 1.57))
    rospy.logerr("Lugar Alcanzado!")
    mic.mover_robot_a_destino((0, 0, 0))
    rospy.logerr("Lugar Alcanzado!")
