#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math
from Phase1ProcessingLib.py import *


rospy.("laserscan_to_pointcloud")
lp = lg.LaserProjection()

def scan_cb(msg):
     global lp
    print("execute call back!!!")
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    print("Finished code line 1: ", pc2_msg)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    print("Finished code line 2: ", point_generator)

    # get list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)
    print("Finished code line 3: ", point_list)

    #print(point_list)
    while 1:
        pass


if __name__ == "__main__":S
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    exit(0)
