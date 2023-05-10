import rospy
from std_msgs.msg import Float64


class VirtualRobot(object):

    def __init__(self):
        rospy.init_node('VirtualRobot')

        self.x_pos = 0.0
        self.speed = 0.0

        self.dist_set_point = rospy.Publisher(
            '/robot_dist/setpoint', Float64, queue_size=1)
        while self.dist_set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.dist_state = rospy.Publisher(
            '/robot_dist/state', Float64, queue_size=1)
        while self.dist_state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.actuation = rospy.Subscriber(
            '/robot_dist/control_effort', Float64, self.dist_actuation)

        self.curr_time = 0.0
        self.period = 0.1
        rospy.Timer(rospy.Duration(self.period), self.virtual_odom)

    def dist_actuation(self, data):
        self.speed = float(data.data)
        rospy.loginfo('speed received: %f' % (self.speed))

    def virtual_odom(self, data):
        self.x_pos += self.speed*(self.period)
        self.dist_state.publish(self.x_pos)
        rospy.loginfo('current position: %f' % (self.x_pos))

    def move_forward(self, set_point):
        self.dist_set_point.publish(set_point)


if __name__ == '__main__':

    vrobot = VirtualRobot()
    vrobot.move_forward(3.0)
    rospy.spin()
