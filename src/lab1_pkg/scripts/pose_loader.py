#!/usr/bin/env python3

import rospy
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray


def leer_archivo(nombre_archivo):
    with open(nombre_archivo, 'r') as archivo:
        lineas = archivo.readlines()
    poses = []
    for linea in lineas:
        valores = linea.strip().split(',')
        x = float(valores[0])
        y = float(valores[1])
        w = float(valores[2])
        poses.append([x, y, w])
    return poses


class PoseLoader(object):

    def __init__(self, nombre_archivo):
        rospy.init_node('pose_loader')
        self.cmd_pose_pub = rospy.Publisher(
            '/goal_list', PoseArray, queue_size=10)
        self.puntos = leer_archivo(nombre_archivo)
        self.rate_hz = 0.1
        self.rate = rospy.Rate(self.rate_hz)

    def mover_robot(self):
        for cosa in self.puntos:
            obj = PoseArray()
            obj.poses.append(cosa[0])
            obj.poses.append(cosa[1])
            obj.poses.append(cosa[2])
            print(obj.poses)
            self.cmd_pose_pub.publish(obj)
            self.rate.sleep()


if __name__ == '__main__':

    nombre_archivo = "/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/goal_list.txt"
    pose = PoseLoader(nombre_archivo)
    pose.mover_robot()
