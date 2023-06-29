#!/usr/bin/env python3

import rospy
from scipy import spatial
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import numpy as np
from random import gauss
from pf_map import pub_initial_pose


class Lidar(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.x_hip = 0
        self.new_time = 0
        self.old_time = 0
        self.y_hip = 0
        self.yaw = 0
        self.lineal_speed = 0
        self.angular_speed = 0
        self.sigma_hit = 0.3
        self.map = None
        rospy.init_node('lidar')
        self.rate_hz = 10
        self.mapa_local = OccupancyGrid()
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        self.map_sub = rospy.Subscriber('/pf_map', OccupancyGrid, self.mapa_cb)
        self.mover = rospy.Publisher(
            '/mover', PoseArray, queue_size=10)
        self.scaner_q = rospy.Publisher(
            '/scaner_q', OccupancyGrid, queue_size=10)
        self.__particle_sub = rospy.Subscriber(
            "particles", PoseArray, queue_size=2)
        rospy.sleep(2)
        self.q = {}

    def particle_cb(self, data):
        # store particles
        pass

    def move_particles(self, where):
        # move particles and assign sigma value
        pass

    def mapa_cb(self, data):
        self.resolution = data.info.resolution
        self.width = data.info.width
        self.height = data.info.height
        origin = data.info.origin

        self.mapa_local.header = Header(
            stamp=rospy.Time.now(), frame_id='pf_map_frame')
        self.mapa_local.info = data.info
        self.mapa_local.data = [1 for _ in range(self.width * self.height)]

        grid = np.array(data.data).reshape(self.height, self.width)

        self.points = [[x * self.resolution + origin.position.x, y * self.resolution + origin.position.y]
                       for y in range(self.height) for x in range(self.width) if grid[y, x] > 50]
        pub_initial_pose(x=origin.position.x, y=origin.position.y, yaw=0.0)

        # KDTree del mapa real.
        # a KDTree hay que pasarle los obstáculos

        self.map = spatial.KDTree(self.points)
        self.q = {}

    def find_closest_point(self, point):
        distance, index = self.map.query(point)
        # rospy.loginfo(f'distance: {distance} index: {index}')
        coordenadas = self.points[index]
        # rospy.loginfo(f'coordenadas: {coordenadas}')
        return distance, coordenadas

    def lidar_cb(self, data):
        self._range_max = data.range_max
        self._range_min = data.range_min
        self._ang_increment = data.angle_increment
        self._ang_min = data.angle_min
        self._ang_max = data.angle_max

        self.ranges = data.ranges
        self.dist = []
        for theta, valor in enumerate(self.ranges):
            # obtener particulas hipoteticas
            # a cada particula se le asigna un peso
            self.likelyhood_fields(theta, valor)
            # reescribo las poses hipoteticas como el valor verdadero del coso.

        # HACE QUE SE MUEVA EL ROBOT CADA 5 SEGUNDOS
        # la idea de esto es que se detenga cada cierto avance para tomar datos
        self.new_time = int(rospy.get_time())
        if (self.new_time - self.old_time) == 5:
            # pose_array_msg = PoseArray()
            # self.se_mueve = 1
            # pose_array_msg.header.frame_id = str(self.se_mueve)
            # self.mover.publish(pose_array_msg)
            pass

        if (self.new_time - self.old_time) == 10 or self.old_time == 0:
            self.old_time = self.new_time
            # pose_array_msg = PoseArray()
            # self.se_mueve = 0
            # pose_array_msg.header.frame_id = str(self.se_mueve)
            # self.mover.publish(pose_array_msg)
            self.lidar_pub()

    def lidar_pub(self):
        self.scaner_q.publish(self.mapa_local)

    def likelyhood_fields(self, theta, valor):
        if self.map is not None and valor < 4:
            angulo = self._ang_min + theta*self._ang_increment

            x = self.x + valor * np.cos(self.yaw + angulo)
            y = self.y + valor * np.sin(self.yaw + angulo)
            # rospy.sleep(5)
            distance, coordenadas = self.find_closest_point((x, y))
            # rospy.loginfo(f'punto: {punto}')
            dist = round(
                np.sqrt(((x - coordenadas[0])**2) + ((y - coordenadas[1])**2)), 3)
            # rospy.loginfo(f'dist: {dist}')
            prob = gauss(dist, self.sigma_hit)
            # rospy.loginfo(f'prob: {prob}')
            q_new = 1 * prob

            if theta in self.q.keys():
                q_old = self.q[theta][0]
                q_new = q_old * prob
                self.q[theta] = [q_new, (x, y)]
                # OBTENEER OCCUPANCY MAP PARA MAPA_LOCAL
                x = int((x)/self.resolution)
                y = int((y)/self.resolution)
                if 0 <= x < self.mapa_local.info.width - 1 and 0 <= y < self.mapa_local.info.height - 1:
                    pos = y * self.mapa_local.info.width + x
                    if pos < len(self.mapa_local.data) - 1:
                        self.mapa_local.data[pos] = int(q_new * 100)

            else:
                self.q[theta] = [q_new, (x, y)]


if __name__ == '__main__':

    lidar = Lidar()
    rospy.spin()
