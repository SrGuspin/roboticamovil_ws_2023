#!/usr/bin/env python3

import rospy
from scipy import spatial
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import numpy as np
from random import gauss
from pf_map import PFMap, pub_initial_pose


class Lidar(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.lineal_speed = 0
        self.angular_speed = 0
        self.sigma_hit = 0.3
        rospy.init_node('lidar')
        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        self.map_sub = rospy.Subscriber('/pf_map', OccupancyGrid, self.mapa_cb)
        rospy.sleep(2)
        self.map = None
        self.q = []

    def mapa_cb(self, data):
        resolution = data.info.resolution
        width = data.info.width
        height = data.info.height
        origin = data.info.origin
        rospy.loginfo(f'resolution: {resolution}')

        grid = np.array(data.data).reshape(height, width)

        self.points = [[x * resolution + origin.position.x, y * resolution + origin.position.y]
                       for y in range(height) for x in range(width) if grid[y, x] > 50]

        # KDTree del mapa real.
        # a KDTree hay que pasarle los obstáculos

        # pub_initial_pose(x=0.5, y=0.5, yaw=0.0)
        # map = PFMap("pf_map")
        # rospy.loginfo(f'mapita {map}')
        # wall_positions = map.get_walls_positions()
        # rospy.loginfo(f'wall_positions {wall_positions}')

        self.map = spatial.KDTree(self.points)
        rospy.loginfo(f'self.mapa: {self.map}')
        self.q = []

    def find_closest_point(self, point):
        distance, index = self.map.query(point)
        # rospy.loginfo(f'distance: {distance} index: {index}')
        coordenadas = self.points[index]
        # rospy.loginfo(f'coordenadas: {coordenadas}')
        return distance, coordenadas
        # closest_point = self.map[index]
        # return closest_point
        # return distance

    def lidar_cb(self, data):
        self._range_max = data.range_max
        self._range_min = data.range_min
        self._ang_increment = data.angle_increment
        self._ang_min = data.angle_min
        self._ang_max = data.angle_max
        # 180 datos
        # rospy.loginfo(f'numero de datos: {len(data.ranges)}')

        # iterar haz por haz del lidar
        # luego llamar a likelyhood fields con el valor del haz

        self.ranges = data.ranges
        self.q = []
        self.dist = []
        for theta, valor in enumerate(self.ranges):
            self.likelyhood_fields(theta, valor)
            # rospy.loginfo(f'thetha: {theta}')
        rospy.loginfo(f'promedio de q: {np.mean(self.q)}')
        # rospy.loginfo(f'desviación estandar: {np.std(self.dist)}')

    def likelyhood_fields(self, theta, valor):
        if self.map is not None and valor < 4:
            if theta < 90:
                self.theta_real = -(90-theta)*self._ang_increment
            else:
                self.theta_real = (theta-90)*self._ang_increment

            x = self.x + valor * np.cos(self.yaw + self.theta_real)
            y = self.y + valor * np.sin(self.yaw + self.theta_real)
            # rospy.sleep(5)
            distance, coordenadas = self.find_closest_point((x, y))
            # rospy.loginfo(f'punto: {punto}')
            dist = round(
                np.sqrt(((x - coordenadas[0])**2) + ((y - coordenadas[1])**2)), 3)
            self.dist.append(dist)
            # rospy.loginfo(f'dist: {dist}')
            prob = gauss(dist, self.sigma_hit)
            # rospy.loginfo(f'prob: {prob}')
            q_new = 1 * prob

            if len(self.q) == len(self.ranges):
                q_old = self.q[theta]
                q_new = q_old * prob

                self.q[theta] = q_new
            else:
                self.q.append(q_new)

            # rospy.loginfo(f'q_new: {q_new}')

        # rospy.loginfo(f'Nuevo theta: {self.theta_real}')
        # x = self.x + valor * np.cos(self.yaw + theta*self._ang_increment)
        # y = self.y + valor * np.sin(self.yaw + theta*self._ang_increment)
        # punto = self.find_closest_point((x, y))
        # dist = np.sqrt(((x - punto[0])**2) + ((y - punto[1])**2))
        # prob = gauss(dist, self.sigma_hit)
        # q_new = 1 * prob

        # if len(self.q) == len(self.ranges):
        #     q_old = self.q[theta]
        #     q_new = q_old * prob

        #     self.q[theta] = q_new
        # else:
        #     self.q.append(q_new)
        #     print(q_new)


if __name__ == '__main__':

    lidar = Lidar()
    rospy.spin()
