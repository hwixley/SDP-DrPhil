#!/usr/bin/env python3

import rospy
from time import time

from devices.Lidar import Lidar
from robots.waffle import Waffle

import sys

class Webots_bridge:
    def __init__(self,robot_name):
        self.r_name = robot_name

        self.robot = Waffle(self.r_name)


    def setup(self):
        self.robot.setup()

    def update(self,dt):
        self.robot.update(dt)

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print("You need to run this node with <robot unique name> argument!")
    else:
        rospy.init_node('webots_bridge',anonymous=True)
        
        node = Webots_bridge(sys.argv[1])
        rate = rospy.Rate(15.625) # ROS Rate at 15hz ~ 64ms time step
        
        # setup devices 
        node.setup()

        # publish messages consistently untill shutdown
        while not rospy.is_shutdown():
            try:
                node.update(1./15.625)
                rate.sleep()
            except Exception as e:
                print("Exception in webots bridge: " + str(e))
                pass
