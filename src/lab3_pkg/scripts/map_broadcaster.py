#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from tf import transformations, TransformBroadcaster


class ParticleFilterMap(object):
    def __init__(self):
        self.variable_init()
        self.connections_init()

    def variable_init(self):
        self.br = TransformBroadcaster()
        pass

    def connections_init(self):
        self.pub_map = rospy.Publisher('pf_map', OccupancyGrid, queue_size=10)
        self.sub_map = rospy.Subscriber('map', OccupancyGrid, self.tf_map)

    def tf_map(self, map_msg):
        width, height = map_msg.info.width, map_msg.info.height
        np_map = np.array(map_msg.data)
        np_map = np_map.reshape((height, width))
        np_map = np.flip(np_map, 0)
        np_map = np_map.reshape((height*width))

        map_msg.header.frame_id = "pf_map_frame"
        map_msg.data = np_map.tolist()
        map_msg.info.origin.position.x, map_msg.info.origin.position.y = 0, 0
        map_msg.info.map_load_time = rospy.Time.now()

        self.br.sendTransform((-0.5, -0.5, 0),
                              transformations.quaternion_from_euler(0, 0, 0),
                              map_msg.info.map_load_time,
                              "pf_map_frame",
                              "odom")

        self.pub_map.publish(map_msg)


if __name__ == '__main__':
    rospy.init_node('particle_filter_map')
    kobuki_sim = ParticleFilterMap()
    rospy.spin()
