#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np


class Follower:

    def __init__(self):
        rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
        rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
        self.pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)
        self.target = (0, 0)
        
    def callback1(self, msg):
        self.target = (msg.x, msg.y)
        
    def callback2(self, msg):
        cur_pose = (msg.x, msg.y, msg.theta)
        msg = Twist()
        
        if abs(cur_pose[0] - self.target[0]) + abs(cur_pose[1] - self.target[1]) < 0.01:
            self.pub.publish(msg)
            return
        
        if cur_pose[0] == self.target[0]:
            if cur_pose[1] > self.target[1]:
                target_theta = - np.pi / 2
            else:
                target_theta = np.pi / 2
        else:
            target_theta = np.arctan((cur_pose[1] - self.target[1]) / (cur_pose[0] - self.target[0]))
            if cur_pose[1] < self.target[1] and cur_pose[0] > self.target[0]:
                target_theta += np.pi
            elif cur_pose[1] > self.target[1] and cur_pose[0] > self.target[0]:
                target_theta -= np.pi
                
        #rospy.loginfo(f'{cur_pose[2]}  {target_theta}')
        
        delta = target_theta - cur_pose[2]
        if delta > np.pi:
            delta -= 2 * np.pi
        elif delta < - np.pi:
            delta += 2 * np.pi
        if abs(delta) < 0.5:
            msg.linear.x = 1
        if abs(delta) > 0.001:
            msg.angular.z = delta / abs(delta) * (1 if abs(delta) > 0.05 else 0.1)
            
        self.pub.publish(msg)
        

rospy.init_node('turtle_follower')
Follower()
rospy.spin()
