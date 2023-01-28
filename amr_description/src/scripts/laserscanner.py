#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    i = max(msg.intensities)
    d = msg.ranges[180]
    print ("intensities: ",i, "distance: ",d)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/front_scan', LaserScan, callback)
rospy.spin()