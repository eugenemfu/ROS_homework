#!/usr/bin/python3

# HW2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np


class Filter:

    def __init__(self):
        rospy.Subscriber('/filtered_scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.r = rospy.Rate(0.2)
        
    def callback(self, scan):
    
        # you can change these parameters:
        map_size = 80
        res = 0.25
        
        radius = map_size // 2
        data = np.zeros((map_size, map_size), dtype=np.int8)
        data[0, 1] = 100
        
        distances = np.array(scan.ranges)
        angle_range = (scan.angle_min, scan.angle_max)
        angle_num = distances.shape[0]
        angles = np.linspace(*angle_range, angle_num)
        
        for i in range(angle_num):
            x = distances[i] * np.sin(angles[i])
            x = int(x / res) + radius
            y = distances[i] * np.cos(angles[i])
            y = int(y / res) + radius
            
            if 0 <= x < map_size and 0 <= y < map_size: 
                data[x, y] = 100
        
        msg = OccupancyGrid(data=data.flatten().tolist())
        msg.info = MapMetaData()
        msg.info.height = data.shape[0]
        msg.info.width = data.shape[1]
        msg.info.resolution = res
        msg.info.origin = Pose()
        msg.info.origin.position = Point()
        msg.info.origin.position.x = -radius * res
        msg.info.origin.position.y = -radius * res
        msg.header.frame_id = 'base_link'
        
        self.pub.publish(msg)
        

rospy.init_node('map_builder')

r = rospy.Rate(0.5)
r.sleep()

Filter()
rospy.spin()
