#!/usr/bin/env python3

import rospy
from scipy.spatial import KDTree
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np


class Lidar(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.lineal_speed = 0
        self.angular_speed = 0
        rospy.init_node('lidar')
        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        self.map_sub = rospy.Subscriber('pf_map', OccupancyGrid, self.mapa_cb)
        self.particles_sub = rospy.Subscriber(
            'particles', PoseArray, self.particles_cb)
        self.map = None
        self.particles = []

    def mapa_cb(self, data):
        resolution = data.info.resolution
        width = data.info.width
        height = data.info.height
        origin = data.info.origin

        grid = np.array(data.data).reshape(height, width)

        self.points = [(x * resolution + origin.position.x, y * resolution + origin.position.y)
                       for y in range(height) for x in range(width) if grid[y, x] > 50]

        # KDTree del mapa real.
        self.map = KDTree(self.points)

    def find_closest_point(self, point):
        if self.map is None:
            return None
        distance, index = self.map.query(point)
        closest_point = self.points[index]
        return closest_point

    def particles_cb(self, poses):
        # Poses hipoteticas del robot.
        new_points = [(pose.position.x, pose.position.y)
                      for pose in data.poses]
        self.particles = new_points

    def lidar_cb(self, data):
        self._range_max = data.range_max
        self._range_min = data.range_min
        self._ang_increment = data.angle_increment
        self._ang_min = data.angle_min
        self._ang_max = data.angle_max

        # iterar haz por haz del lidar
        # luego llamar a likelyhood fields con el valor del haz
        self.ranges = data.ranges
        for theta, valor in enumerate(self.ranges):
            self.likelyhood_fields(theta, valor)
        pass

    def likelyhood_fields(self, theta, valor):
        # aplicamos el algoritmo en base a sensor
        # poses hipoteticas = self.particles
        # mapa = self.map

        pass


if __name__ == '__main__':

    lidar = Lidar()
    rospy.spin()
algo
