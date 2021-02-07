#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import time

class ProximityLDS():
    def __init__(self):    
        # input
        self.input_sub = rospy.Subscriber('scan',LaserScan,self.callback) 

    #TODO: Given an angle and range of nearest obstacle, instruct the robot to go to it
    def move_to_closest_obstacle(self,range,angle):
        pass
    

    def move_motor(self,fwd,ang):
        pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
        mc = Twist()
        mc.linear.x = fwd
        mc.angular.z = ang
        pub.publish(mc)


    #Test to move it 
    def move_example(self):
        forward_speed = 1   
        turn_speed = 1
        self.move_motor(forward_speed,turn_speed)

     
     

    def callback(self,data):
        self.move_example()
        print(len(data.ranges))
        min = 3.6
        minIndex = 0 
        for i in range(0,len(data.ranges)):
            if data.ranges[i] < min:
                min = data.ranges[i]
                minIndex = i
        print(minIndex)
        print(min)


# when started as a script
if __name__ == '__main__':

    # start up a new node
    rospy.init_node('proximity',anonymous=True)

    # initialize behaviour
    proximity = ProximityLDS()

    # update at 10 hz untill shutdown
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        rate.sleep()

  
