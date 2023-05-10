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


class Movement(object):

    def __init__(self):
        self.max_w = 1.0  # [rad/s]
        self.max_v = 0.2  # [m/s]
        self.factor_correcion_1 = 1.1
        self.factor_correcion_2 = 1.01
        self.lineal = 0
        self.angular = 0
        self.eje = False
        rospy.init_node('dead_reckoning_nav')
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
        self.mover_sub = rospy.Subscriber(
            '/goal_list', PoseArray, self.accion_mover_cb)

    # Funcion del nivel 1.

    def velocidad_angular(self, data):
        self.angular = float(data.data)

    def velocida_lineal(self, data):
        self.lineal = float(data.data)

    def aplicar_velocidad(self):
        speed = Twist()
        while not rospy.is_shutdown():
            speed.linear.x = self.lineal
            speed.angular.z = self.angular
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

        if self.eje:
            self.lineal_state.publish(self.x)
        else:
            self.lineal_state.publish(self.y)
        speed = Twist()
        if not rospy.is_shutdown():
            speed.linear.x = self.lineal
            speed.angular.z = self.angular
            self.cmd_vel_mux_pub.publish(speed)
            self.rate_obj.sleep()

    # Funcion del nivel 2

    def mover_robot_a_destino(self, goal_pose):
        # Alinear con destino

        primero_mov = (goal_pose[0], 0)
        segundo_mov = (0, goal_pose[2])
        tercero_mov = (goal_pose[1], 0)
        lista_desplazamientos = [primero_mov, segundo_mov, tercero_mov]

        self.eje = True
        self.ang_set_point.publish(primero_mov[1])
        self.lin_set_point.publish(primero_mov[0])

        self.ang_set_point.publish(segundo_mov[1])
        self.lin_set_point.publish(segundo_mov[0])

        self.eje = False
        self.lin_set_point.publish(tercero_mov[0])
        rospy.logerr(123123123123)

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

    mic = Movement()
    mic.x = 0
    mic.y = 0
    mic.yaw = 0
    rospy.spin()
