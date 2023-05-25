#! /usr/bin/env python3
# Hasta aqui funciona la llegada pero es reactivo en su movimiento

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist, PoseArray, Pose
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# from turtlebot_audio import TurtlebotAudio

from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


def getAngle(a, b, c):
    p1 = [a[0]-b[0], a[1]-b[1]]
    p2 = [c[0]-b[0], c[1]-b[1]]
    ang = math.atan2(p2[1], p2[0]) - math.atan2(p1[1], p1[0])

    if ang >= math.pi:
        ang -= 2*math.pi
    elif ang < -math.pi:
        ang += 2*math.pi

    return ang


class TurtlebotController(object):

    def __init__(self):
        # self.speaker = TurtlebotAudio()
        self.bridge = CvBridge()
        self.vector = Vector3(0, 0, 0)
        self.depth_image_np = None
        self.error_distancia = 0
        self.resolucion = 0.01

        self.ref_ang = 0
        self.angular_speed = 0
        self.pos = 0
        self.eje = False

        self.rate_hz = 10
        self.rate_obj = rospy.Rate(self.rate_hz)
        # rospy.init_node('turtlebot')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.depth_img_sub = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.depth_image_cb)
        self.cmd_vel_pub = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.occupancy_state_pub = rospy.Publisher(
            '/occupancy_state', Vector3, queue_size=10)

        # ESTADOS PID
        self.ang_set_point = rospy.Publisher(
            '/robot_angular/setpoint', Float64, queue_size=1)

        self.angular_state = rospy.Publisher(
            '/robot_angular/state', Float64, queue_size=1)

        self.ang_actuation = rospy.Subscriber(
            '/robot_angular/control_effort', Float64, self.velocidad_angular)

        while self.angular_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        while self.ang_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        self.ang_set_point.publish(0)

        # self.mover_sub = rospy.Subscriber(
        #    '/goal_list', PoseArray, self.accion_mover_cb)

        rospy.sleep(0.2)

    def velocidad_angular(self, data):
        self.angular_speed = float(data.data)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        roll, pitch, self.yaw = euler_from_quaternion((msg.pose.pose.orientation.x,
                                                       msg.pose.pose.orientation.y,
                                                       msg.pose.pose.orientation.z,
                                                       msg.pose.pose.orientation.w))
        rospy.loginfo('odometría: (%f,%f,%f)' %
                      (round(self.x, 3), round(self.y, 3), round(self.yaw, 3)))

        if self.yaw < 0:
            self.yaw = self.yaw + 2*np.pi
        self.angular_state.publish(self.yaw)  # donde está

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
            columnas = Vector3(columna1, columna2, columna3)
            self.centro_tunel = (np.mean(columna3) - np.mean(columna1))/2
            self.pos_vs_centro = abs(self.y - 1.25) - self.centro_tunel
            rospy.loginfo('columna1: %f' % np.mean(columna1))
            rospy.loginfo('columna2: %f' % np.mean(columna3))
            rospy.loginfo('centro tunel: %f' % self.centro_tunel)
            rospy.loginfo('diferencia a referencia: %f' % self.pos_vs_centro)

            frente = (self.x+np.cos(self.yaw), self.y+np.sin(self.yaw))
            angulo = getAngle(frente, (self.x, self.y),
                              (self.x, self.centro_tunel))
            obj_aux_yaw_1 = self.yaw + angulo  # MANDAR A PID
            # columna 1 = izquierda
            # columna 2 = frente
            # columna 3 = derecha
            # rospy.loginfo("columna1: {}".format(np.mean(columna1)))

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

        ciclos = round(tiempo*self.rate_hz, 3)
        speed = Twist()
        speed.linear.x = lin_speed
        speed.angular.z = self.
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
        giro_der = -0.35
        giro_izq = 0.35
        tiempo_medio_giro = int(np.pi/giro_izq)
        while not rospy.is_shutdown():
            vector = self.vector

            if vector.x == 1 and vector.z == 1:
                goal_pose = self.x, self.yaw, tiempo_medio_giro*10
                self.mover_robot_a_destino(goal_pose)
                # self.aplicar_velocidad((0, 0, tiempo_medio_giro/4))
                # giro = self.detectar_imagen()
                # self.aplicar_velocidad((0, giro, tiempo_medio_giro*2/3))
                # self.aplicar_velocidad((0, 0, tiempo_medio_giro*10))
                # pass
            elif vector.x == 1:
                goal_pose = (self.x+10, self.y, self.yaw-10)
                self.mover_robot_a_destino(goal_pose)
                # self.aplicar_velocidad((0.2, giro_der, tiempo_medio_giro/10))
            elif vector.z == 1:
                goal_pose = (self.x-10, self.y, self.yaw+10)
                self.mover_robot_a_destino(goal_pose)
                # self.aplicar_velocidad((0.2, giro_izq, tiempo_medio_giro/10))
            else:
                goal_pose = self.x+10, self.y, self.yaw
                self.mover_robot_a_destino(goal_pose)
                # elf.aplicar_velocidad((0.2, 0, 1))

    def detectar_imagen(self):
        img = cv2.imread(cv2.samples.findFile(
            '/home/govidal/code/robotica-movil/workspace/src/lab2_pkg/scripts/arrow_left.png'))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100,
                                minLineLength=5, maxLineGap=190)
        sumax = 0
        sumay = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            sumax += x1 + x2
            sumay += y1 + y2

        if sumax > 12000:
            giro = -0.35
        else:
            giro = 0.35
        return giro

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
        # obj_aux_yaw_1 = ajustar a la dirección del objetivo en x
        # obj_x = cuanto se tiene que mover en x
        # obj_aux_yaw_2 = ajustar a la dirección del objetivo en y
        # obj_y = cuanto se tiene que mover en y
        # obj_ang = angulo objetivo desde el inicio
        displacement_list = [obj_aux_yaw_1,
                             obj_x, obj_aux_yaw_2, obj_y, obj_ang]
        self.aplicar_velocidad(displacement_list)


if __name__ == '__main__':

    # mic = TurtlebotController()
    # mic.x = 0
    # mic.y = 0
    # mic.yaw = 0
    # rospy.spin()
    rospy.init_node('reactive_movement')
    turtlebot = TurtlebotController()
    turtlebot.run()
