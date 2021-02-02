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

    def update(self,time):
        self.robot.update(time)

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print("You need to run this node with <robot unique name> argument!")
    else:
        time = 0
        rospy.init_node('webots_bridge',anonymous=True)
        
        node = Webots_bridge(sys.argv[1])
        rate = rospy.Rate(15.625) # ROS Rate at 15hz ~ 64ms time step
        
        # setup devices 
        node.setup()

        # publish messages consistently untill shutdown
        while not rospy.is_shutdown():
            # try:
            step = 1./15.625
            time += step
            node.update(time)
            rate.sleep()
                # except Exception as e:
                #     print("Exception in webots bridge: " + str(e))
                #     pass
