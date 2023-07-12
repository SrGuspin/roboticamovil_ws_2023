#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import math


def goal_done(status, result):
    rospy.loginfo('DONE status: %s (%s)',
                  move_base_client.get_goal_status_text(), str(status))


if __name__ == '__main__':

    rospy.init_node('nav_stack_example')
    move_base_client = actionlib.SimpleActionClient(
        '/move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    positions = [(15.64, 3.603, 0), (25.811, 9.083, 0), (7.69, 11.9577, 0)]

    for pos in positions:
        quaternion = tf.transformations.quaternion_from_euler(0, 0, pos[2])
        goal_pose = Pose()
        goal_pose.position.x = pos[0]
        goal_pose.position.y = pos[1]
        goal_pose.orientation.x = quaternion[0]
        goal_pose.orientation.y = quaternion[1]
        goal_pose.orientation.z = quaternion[2]
        goal_pose.orientation.w = quaternion[3]

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = goal_pose
        move_base_client.send_goal(move_base_goal, done_cb=goal_done)

        # Wait for this goal to complete before going to next position
        move_base_client.wait_for_result()

    rospy.spin()
