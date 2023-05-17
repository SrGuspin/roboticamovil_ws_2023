#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose
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


class TurtleBot(object):

    def __init__(self):
        self.ref_lin = 0
        self.ref_ang = 0
        self.nombre_archivo_odom = '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/data/PI/odom_1.txt'
        self.nombre_archivo_angulo = '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/data/PI/ang_1.txt'
        self.nombre_archivo_linealx = '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/data/PI/linx_1.txt'
        self.nombre_archivo_linealy = '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/data/PI/liny_1.txt'
        with open(self.nombre_archivo_odom, 'w') as archivo:
            archivo.write("0,0\n")
        with open(self.nombre_archivo_angulo, 'w') as archivo:
            archivo.write("0,0,0\n")
        with open(self.nombre_archivo_linealx, 'w') as archivo:
            archivo.write("0,0,0\n")
        with open(self.nombre_archivo_linealy, 'w') as archivo:
            archivo.write("0,0,0\n")
        self.max_w = 1.0  # [rad/s]
        self.max_v = 0.2  # [m/s]
        self.factor_correcion_1 = 1.1
        self.factor_correcion_2 = 1.01
        self.lineal_speed = 0
        self.angular_speed = 0
        self.pos = 0
        self.eje = False
        rospy.init_node('turtlebot')
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

        self.lin_set_point = rospy.Publisher(
            '/robot_lineal/setpoint', Float64, queue_size=1)

        self.lineal_state = rospy.Publisher(
            '/robot_lineal/state', Float64, queue_size=1)

        self.lin_actuation = rospy.Subscriber(
            '/robot_lineal/control_effort', Float64, self.velocida_lineal)

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
            '/goal_list', PoseArray, self.accion_mover_cb)

        rospy.sleep(0.2)

    def velocidad_angular(self, data):
        self.angular_speed = float(data.data)

    def velocida_lineal(self, data):
        self.lineal_speed = float(data.data)

    def aplicar_velocidad(self, displacement_list):
        for i, displacement in enumerate(displacement_list):
            if i == 1:
                print(1)
                self.eje = True
                self.pos = self.x
                self.ref_lin = displacement[0]
                dist = abs(self.x) + abs(displacement[0])
                self.lin_set_point.publish(dist)
                condicional = True
            elif i == 3:
                self.eje = False
                self.pos = self.y
                self.ref_lin = displacement[0]
                dist = abs(self.y) + abs(displacement[0])
                self.lin_set_point.publish(dist)
                condicional = True
                print(2)
            else:
                self.ref_ang = displacement[1]
                self.ang_set_point.publish(displacement[1])
                condicional = False
                print(3)

            rospy.sleep(0.5)
            speed = Twist()
            while True:
                if not condicional:
                    speed.linear.x = 0
                    if round(self.angular_speed, 3) == 0:
                        break
                else:
                    speed.linear.x = self.lineal_speed
                    if round(self.lineal_speed, 5) == 0:
                        break
                speed.angular.z = self.angular_speed
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
        self.angular_state.publish(self.yaw)
        with open(self.nombre_archivo_odom, 'a') as archivo:
            archivo.write(f"{self.x},{self.y}\n")
        with open(self.nombre_archivo_angulo, 'a') as archivo:
            archivo.write(
                f"{self.ref_ang},{self.angular_speed},{self.yaw}\n")

        if self.eje:
            with open(self.nombre_archivo_linealx, 'a') as archivo:
                archivo.write(
                    f"{self.ref_lin},{self.lineal_speed},{self.x}\n")
            with open(self.nombre_archivo_linealy, 'a') as archivo:
                archivo.write(f"{0},{0},{self.y}\n")
            self.lineal_state.publish(abs(self.x - self.pos))
        else:
            with open(self.nombre_archivo_linealy, 'a') as archivo:
                archivo.write(
                    f"{self.ref_lin},{self.lineal_speed},{self.y}\n")
            with open(self.nombre_archivo_linealx, 'a') as archivo:
                archivo.write(f"{0},{0},{self.x}\n")
            self.lineal_state.publish(abs(self.y - self.pos))

    # Funcion del nivel 2

    def mover_robot_a_destino(self, goal_pose):
        # Alinear con destino

        obj_x = (goal_pose[0], 0)
        obj_ang = (0, goal_pose[2])
        obj_y = (goal_pose[1], 0)

        # primer movimiento por eje eje x
        # ajustar hacia direccion del objetivo
        frente = (self.x+np.cos(self.yaw), self.y+np.sin(self.yaw))
        angulo = getAngle(frente, (self.x, self.y), (obj_x[0], self.y))
        obj_aux_yaw_1 = (0, self.yaw + angulo)
        # mover hacia eje

        # Segundo movimiento corresponde a el movimiento en el eje y
        # Ajustar posicion
        x = goal_pose[0]
        frente = (x+np.cos(self.yaw), self.y+np.sin(self.yaw))
        angulo = getAngle(frente, (x, self.y), (x, obj_y[0]))
        obj_aux_yaw_2 = (0, self.yaw + angulo)
        # mover hacia posicion

        # calcualr diferencia de angulo
        # angulo actual - angulo objetivo
        # ajustar direccion angular.

        displacement_list = [obj_aux_yaw_1,
                             obj_x, obj_aux_yaw_2, obj_y, obj_ang]
        self.aplicar_velocidad(displacement_list)

    # Funcion del nivel 3

    def accion_mover_cb(self, algo):
        poses = algo.poses
        for i in poses:
            x = i.position.x
            y = i.position.y
            yaw = i.orientation.w
            rospy.loginfo((x, y, yaw))
            self.mover_robot_a_destino((x, y, yaw))


if __name__ == '__main__':

    mic = TurtleBot()
    mic.x = 0
    mic.y = 0
    mic.yaw = 0
    rospy.spin()
