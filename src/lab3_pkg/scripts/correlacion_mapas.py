#!/usr/bin/env python3
import rospy
from scipy import spatial
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from random import gauss
from pf_map import PFMap, pub_initial_pose


class Correlacion(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        rospy.init_node('correlacion')
        self.global_map = np.array([(1, 1, 1), (0, 0, 0), (0, 0, 0)])
        # self.local_map = np.array([(0, 0, 1), (0, 0, 1), (0, 0, 0)])
        self.__map_l = rospy.Subscriber(
            '/scaner_q', OccupancyGrid, self.mapa_local_cb)
        self.__map_cb = rospy.Subscriber(
            '/pf_map', OccupancyGrid, self.mapa_local_cb)

    def mapa_local_cb(self, data):
        self.mapa_local = data.data

    def mapa_global_cb(self, data):
        resolution = data.info.resolution
        width = data.info.width
        height = data.info.height
        origin = data.info.origin

        grid = np.array(data.data).reshape(height, width)

        self.points = [[x * resolution + origin.position.x, y * resolution + origin.position.y]
                       for y in range(height) for x in range(width) if grid[y, x] > 50]

        # KDTree del mapa real.
        # a KDTree hay que pasarle los obstáculos
        self.map = spatial.KDTree(self.points)

        self.x = origin.position.x
        self.y = origin.position.y

    def correlacionar_mapas(self):
        for angle in [0, 90, 180, 270]:
            rows, cols = self.local_map.shape
            M = cv2.getRotationMatrix2D(
                ((cols - 1)/2.0, (rows - 1)/2.0), angle, 1)
            local_map_rtd = cv2.warpAffine(self.local_map, M, (cols, rows))
            rospy.loginfo(self.global_map)
            rospy.loginfo(self.local_map_rtd)
            corr = cv2.matchTemplate(
                self.global_map, local_map_rtd, cv2.TM_CCOEFF_NORMED)
            rospy.loginfo(f'correlacion matriz {corr}')


if __name__ == '__main__':

    correlacion = Correlacion()
    rospy.spin()
