
import rospy
from scipy import spatial
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from random import gauss
from pf_map import PFMap, pub_initial_pose


class Correlacion(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.global_map = np.array([(1, 1, 1), (0, 0, 0), (0, 0, 0)])
        self.local_map = np.array([(0, 0, 1), (0, 0, 1), (0, 0, 0)])

    def correlacionar_mapas(self):
        for angle in [0, 90, 180, 270]:
            rows, cols = self.local_map.shape
            M = cv2.getRotationMatrix2D(
                ((cols - 1)/2.0, (rows - 1)/2.0), angle, 1)
            local_map_rtd = cv2.warpAffine(self.local_map, M, (cols, rows))
            rospy.loginfo(self.global_map)
            rospy.loginfo(self.local_map_rtd)
            corr = cv2.matchTemplate(
                self.global_map, local_map_rtd, cv2.TM_CCOEFF_NORMED)
            rospy.loginfo(f'correlacion matriz {corr}')


if __name__ == '__main__':

    correlacion = Correlacion()
    rospy.spin()
