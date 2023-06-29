#!/usr/bin/env python3

import rospy
from scipy import spatial
from geometry_msgs.msg import PoseArray, Pose, Twist
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import numpy as np

from random import gauss, choices
from particle import Particle


class Lidar(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.speed = Twist()
        self.x_hip = 0
        self.new_time = 0
        self.old_time = 0
        self.y_hip = 0
        self.yaw = 0
        self.lineal_speed = 0
        self.angular_speed = 0
        self.sigma_hit = 0.01
        self.map = None
        self.particulas = None
        self.sigma = 0.2
        self.q = {}
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
            "particles", PoseArray, self.particle_cb, queue_size=2)

        self.pub_particules = rospy.Publisher(
            "/pf_location", PoseArray, queue_size=10)
        self.velocity_pub = rospy.Subscriber(
            '/yocs_cmd_vel_mux/output/cmd_vel', Twist, self.velocity_cb, queue_size=10)

        rospy.sleep(1)

    def velocity_cb(self, msg):
        # This method is called whenever a new velocity message is received
        # Store the velocity for later use
        self.robot_velocity = msg
        self.update_particle_positions()

    def update_particle_positions(self):
        # This method should be called periodically to update the particle positions
        if self.robot_velocity is not None and self.particulas is not None:
            dt = 1/10  # time since last update
            dtheta = self.robot_velocity.angular.z * dt
            dx = self.robot_velocity.linear.x * np.cos(dtheta) * dt
            dy = self.robot_velocity.linear.x * np.sin(dtheta) * dt

            for i, algo in enumerate(self.particulas):
                self.particulas[i].move(dx, dy, dtheta)
            self.publish_particules()

    def particle_cb(self, data):
        self.particulas = [1 for i in range(3000)]
        self.original_part = [1 for i in range(3000)]
        for n, pose in enumerate(data.poses):
            particle = Particle(pose.position.x, pose.position.y, 0, 0.01, 1)
            self.particulas[n] = particle
            self.original_part[n] = particle
        self.pesos = [1 for i in range(3000)]
        rospy.loginfo("Particulas recibidas!")
        self.publish_particules()
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
        self.x = origin.position.x
        self.y = origin.position.y

        self.points = [[x * self.resolution + origin.position.x, y * self.resolution + origin.position.y]
                       for y in range(self.height) for x in range(self.width) if grid[y, x] > 50]

        self.map = spatial.KDTree(self.points)

    def find_closest_point(self, point):
        distance, index = self.map.query(point)
        # rospy.loginfo(f'distance: {distance} index: {index}')
        coordenadas = self.points[index]
        # rospy.loginfo(f'coordenadas: {coordenadas}')
        return distance, coordenadas

    def lidar_cb(self, data):
        self.new_time = int(rospy.get_time())
        if (self.new_time - self.old_time) == 5:
            pose_array_msg = PoseArray()
            self.se_mueve = 1
            pose_array_msg.header.frame_id = str(self.se_mueve)
            self.mover.publish(pose_array_msg)

        if (self.new_time - self.old_time) == 10 or self.old_time == 0:
            self.old_time = self.new_time
            pose_array_msg = PoseArray()
            self.se_mueve = 0
            pose_array_msg.header.frame_id = str(self.se_mueve)
            self.mover.publish(pose_array_msg)

        if self.particulas is not None and self.se_mueve == 0:
            self._range_max = data.range_max
            self._range_min = data.range_min
            self._ang_increment = data.angle_increment
            self._ang_min = data.angle_min
            self._ang_max = data.angle_max

            self.ranges = data.ranges

            for theta, valor in enumerate(self.ranges):
                # obtener particulas hipoteticas
                # a cada particula se le asigna un peso
                n_part = choices([i for i in range(3000)], self.pesos, k=1)
                particula = self.particulas[n_part[0]]
                w = self.likelyhood_fields(theta, valor, particula)
                self.particulas[n_part[0]].peso = w
                self.pesos[n_part[0]] = w
                # reescribo las poses hipoteticas como el valor verdadero del coso.
            pesos_sum = sum(self.pesos)

            # Normaliza cada probabilidad en la lista
            if pesos_sum != 0:
                self.pesos = [peso / pesos_sum for peso in self.pesos]
                new_part = choices(
                    self.particulas, self.pesos, k=len(self.particulas))
                for part in range(len(new_part)):
                    pos0 = new_part[part].pos()
                    pos1 = self.particulas[part].pos()

                    dist_x = pos0[0] - pos1[0]
                    dist_y = pos0[1] - pos1[1]
                    self.particulas[part].move(dist_x, dist_y, 0)
                    self.particulas[part].peso = new_part[part].peso

            self.publish_particules()

    def likelyhood_fields(self, theta, valor, hip):
        if self.map is not None and valor < 4:
            pos = hip.pos()
            q_old = hip.peso
            angulo = self._ang_min + theta*self._ang_increment

            x = pos[0] + valor * np.cos(pos[2] + angulo)
            y = pos[1] + valor * np.sin(pos[2] + angulo)
            distance, coordenadas = self.find_closest_point((x, y))
            cosa = np.mean(self.pesos)
            prob = gauss(distance, self.sigma)
            q_new = q_old * prob
            return q_new
        else:
            return 0

    def publish_particules(self):
        # aqui creamos uno poseArray para mostra las particulas en Rviz
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = "pf_map_frame"

        for part in self.particulas:
            part_pose = Pose()
            part_pose.position.x, part_pose.position.y = part.x, part.y
            quat = quaternion_from_euler(0, 0, 0)

            part_pose.orientation.x = quat[0]
            part_pose.orientation.y = quat[1]
            part_pose.orientation.z = quat[2]
            part_pose.orientation.w = quat[3]

            pose_array_msg.poses.append(part_pose)

        pose_array_msg.header.stamp = rospy.Time.now()
        self.pub_particules.publish(pose_array_msg)


if __name__ == '__main__':

    lidar = Lidar()
    rospy.spin()
