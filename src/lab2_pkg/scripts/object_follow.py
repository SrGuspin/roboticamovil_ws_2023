#!/usr/bin/env python3

import rospy
from std_msgs.msg import Point
from geometry_msgs.msg import Twist

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower')
        self.blue_square_sub = rospy.Subscriber('blue_square_position', Point, self.position_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.reference_position = 0
        self.Kp = 0.5  # Ganancia proporcional del controlador P
        
    def position_callback(self, position_msg):
        position_x = position_msg.x
        self.rotate_to_center(position_x)
        
    def rotate_to_center(self, position_x):
        error = self.reference_position - position_x
        
        # Controlador P
        angular_vel = self.Kp * error
        
        # Crear mensaje Twist para controlar el movimiento del robot
        cmd_vel = Twist()
        cmd_vel.angular.z = angular_vel
        cmd_vel.linear.x = 0.2  # Velocidad lineal constante
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    object_follower = ObjectFollower()
    object_follower.run()
