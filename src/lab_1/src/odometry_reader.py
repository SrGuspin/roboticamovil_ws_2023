#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class OdometryReader( object ):

  def __init__( self ):
    rospy.init_node( 'read_odom' )
    self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odometry_cb )

  def odometry_cb( self, odom ):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )
    rospy.loginfo( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % (x, y, z, roll, pitch, yaw) )


if __name__ == '__main__':

  odom_reader = OdometryReader()
  rospy.spin()


