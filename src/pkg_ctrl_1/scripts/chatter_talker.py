#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node('my_first_talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        pass
    rate = rospy.Rate(2)  # 2Hz
    while not rospy.is_shutdown():
        hello_str = 'hello world %s' % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
