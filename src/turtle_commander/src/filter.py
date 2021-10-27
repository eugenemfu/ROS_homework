#!/usr/bin/python3

# HW2

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


class Filter:

    def __init__(self):
        rospy.Subscriber('/base_scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=1)
        
    def callback(self, msg):
        arr = np.array(msg.ranges)
        mask = np.zeros_like(arr, dtype=bool)
        for i in range(1, len(arr) - 1):
            if (abs(arr[i - 1] + arr[i + 1] - 2 * arr[i])) < 0.2:
                mask[i] = True
                
        msg.ranges = tuple((arr * mask).tolist())
        self.pub.publish(msg)
        

rospy.init_node('scan_filter')

r = rospy.Rate(0.2)
r.sleep()

Filter()
rospy.spin()
