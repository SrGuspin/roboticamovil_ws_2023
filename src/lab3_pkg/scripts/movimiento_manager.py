#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseArray, Pose
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid

import numpy as np


class TurtlebotController(object):

    def __init__(self):
        # self.speaker = TurtlebotAudio()
        self.vector = Vector3(0, 0, 0)
        self.depth_image_np = None
        self.se_mueve = 0
        self.rate_hz = 5
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_vel_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        self.mover = rospy.Subscriber('/mover', PoseArray, self.mover_cb)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w))
        # rospy.loginfo('Current pose - lin: (%f, %f) ang: (%f)' %
        #               (x, y, yaw))

    def mover_cb(self, data):
        self.se_mueve = data.header.frame_id
        rospy.loginfo(f"se_mueve : {self.se_mueve}")

    def lidar_cb(self, data):
        self.ranges = data.ranges
        self.valores_scan = []
        for theta, valor in enumerate(self.ranges):
            if theta > 61 and theta < 119:
                self.valores_scan.append(valor)
        # rospy.loginfo(self.valores_scan)
        # rospy.loginfo(len(self.valores_scan))

        self.obstacle_detected()

    def obstacle_detected(self):
        # Es un vector de tres cosas, que incluye la condicion, idea:
        # (0, 0, 0) -> sigue derecho
        # (1, 0, 0) -> rota a la derecha
        # (0, 1, 0) -> Retrocede
        # (0, 0, 1) -> Rota a la izquierda
        # Los otros casos de por ejemplo, retroceder y girar a la vez,
        # Provocaran que  termine chocando al darse la vuelta, ya que,
        # no tenemos sensores en la parte trasera del equipo

        obstacle = False
        if True:

            columna1 = np.mean(self.valores_scan[0:19])
            columna2 = np.mean(self.valores_scan[19:38])
            columna3 = np.mean(self.valores_scan[38:])
            # rospy.loginfo(f"1: {columna1}, 2 {columna2}, 3: {columna3}")

            obstacle1 = columna1 < 0.5
            obstacle2 = columna2 < 0.5
            obstacle3 = columna3 < 0.5

            if obstacle1:
                obstacle1 = 1
            else:
                obstacle1 = 0

            if obstacle2:
                obstacle2 = 1
            else:
                obstacle2 = 0

            if obstacle3:
                obstacle3 = 1
            else:
                obstacle3 = 0

            self.vector = Vector3(obstacle1, obstacle2, obstacle3)
        else:
            self.vector = Vector3(0, 0, 0)
        return self.vector

    def aplicar_velocidad(self, tripleta):
        lin_speed = tripleta[0]
        ang_speed = tripleta[1]
        tiempo = tripleta[2]

        ciclos = round(tiempo*self.rate_hz)
        speed = Twist()
        speed.linear.x = lin_speed
        speed.angular.z = ang_speed
        while not rospy.is_shutdown():
            if ciclos >= 0:
                # rospy.loginfo('publishing speed (%f, %f)' %
                #               (lin_speed, ang_speed))
                self.cmd_vel_pub.publish(speed)
                self.rate_obj.sleep()
            else:
                break
            ciclos -= 1

    def run(self):
        free_space = True
        giro_der = -0.2
        giro_izq = 0.2
        tiempo_medio_giro = int(np.pi/giro_izq)
        while not rospy.is_shutdown():
            if int(self.se_mueve) == 1:
                vector = self.vector
                # (izq, fren, der)
                if vector.x == 1 and vector.z == 1 and vector.y == 1:
                    self.aplicar_velocidad((0, giro_der, tiempo_medio_giro/10))
                # (   , fren, der)
                elif vector.y == 1 and vector.z == 1:
                    self.aplicar_velocidad((0, giro_der, tiempo_medio_giro/15))
                # (   , fren,    )
                elif vector.y == 1 and vector.x == 0 and vector.z == 0:
                    self.aplicar_velocidad((0, giro_der, tiempo_medio_giro/10))
                # (izq,   ,   )
                elif vector.y == 0 and vector.x == 1 and vector.z == 0:
                    self.aplicar_velocidad((0, giro_izq, tiempo_medio_giro/10))
                # (   ,    , der)
                elif vector.y == 0 and vector.x == 0 and vector.z == 1:
                    self.aplicar_velocidad((0, giro_der, tiempo_medio_giro/10))
                # (izq, fren,    )
                elif vector.y == 1 and vector.x == 1:
                    self.aplicar_velocidad((0, giro_izq, tiempo_medio_giro/15))
                else:
                    self.aplicar_velocidad((0.3, 0, 0.1))
            else:
                self.aplicar_velocidad((0, 0, 0))


if __name__ == '__main__':

    rospy.init_node('movimiento_manager')
    turtlebot = TurtlebotController()
    turtlebot.run()
