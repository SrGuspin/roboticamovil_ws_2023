#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist
from tf.transformations import euler_from_quaternion

# from turtlebot_audio import

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sound_play.libsoundplay import SoundClient


class TurtlebotController(object):

    def __init__(self):
        # self.speaker = TurtlebotAudio()
        self.bridge = CvBridge()
        self.sound_handler = SoundClient(blocking=True)
        self.vector = Vector3(0, 0, 0)
        self.depth_image_np = None
        self.rate_hz = 5
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.depth_img_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_image_cb)
        self.cmd_vel_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.occupancy_state_pub = rospy.Publisher(
            '/occupancy_state', Vector3, queue_size=10)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w))
        rospy.loginfo('Current pose - lin: (%f, %f) ang: (%f)' %
                      (x, y, yaw))

    def depth_image_cb(self, msg):
        try:
            self.depth_image_np = self.bridge.imgmsg_to_cv2(msg)
            self.vector = self.obtacle_detected()
        except CvBridgeError as e:
            rospy.logerr(e)

    def obtacle_detected(self):
        # Es un vector de tres cosas, que incluye la condicion, idea:
        # (0, 0, 0) -> sigue derecho
        # (1, 0, 0) -> rota a la derecha
        # (0, 1, 0) -> Retrocede
        # (0, 0, 1) -> Rota a la izquierda
        # Los otros casos de por ejemplo, retroceder y girar a la vez,
        # Provocaran que  termine chocando al darse la vuelta, ya que,
        # no tenemos sensores en la parte trasera del equipo

        obstacle = False
        if self.depth_image_np is not None:
            columna1 = self.depth_image_np[:, 0]
            columna2 = self.depth_image_np[:, 320]
            columna3 = self.depth_image_np[:, 639]

            columna1 = np.where(np.isnan(columna1), 0.0, columna1)
            columna2 = np.where(np.isnan(columna2), 0.0, columna2)
            columna3 = np.where(np.isnan(columna3), 0.0, columna3)

            obstacle1 = np.any(columna1 < 0.5)
            obstacle2 = np.any(columna2 < 0.5)
            obstacle3 = np.any(columna3 < 0.5)

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

            vector = Vector3(obstacle1, obstacle2, obstacle3)
            self.occupancy_state_pub.publish(vector)
        else:
            vector = Vector3(0, 0, 0)
        return vector

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
                rospy.loginfo('publishing speed (%f, %f)' %
                              (lin_speed, ang_speed))
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
        self.sound_handler.say('Hello human, welcome to Robotica Movil')
        while not rospy.is_shutdown():
            vector = self.vector

            if vector.x == 1 and vector.z == 1:
                self.sound_handler.say('obstacle detected')
                self.aplicar_velocidad((0, giro_der, tiempo_medio_giro))
            elif vector.x == 1:
                self.sound_handler.say('obstacle detected')
                self.aplicar_velocidad((0, giro_der, tiempo_medio_giro/10))
            elif vector.z == 1:
                self.sound_handler.say('obstacle detected')
                self.aplicar_velocidad((0, giro_izq, tiempo_medio_giro/10))
            else:
                self.aplicar_velocidad((0.3, 0, 1))


if __name__ == '__main__':

    rospy.init_node('accion_percepcion')
    turtlebot = TurtlebotController()
    turtlebot.run()
