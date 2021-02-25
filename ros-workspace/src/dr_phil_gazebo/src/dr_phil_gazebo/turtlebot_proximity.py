#!/usr/bin/env python3

from __future__ import absolute_import
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import time

class ProximityLDS(object):
    def __init__(self):    
        # input
        self.input_sub = rospy.Subscriber(u'scan',LaserScan,self.callback) 

    #TODO: Given an angle and range of nearest obstacle, instruct the robot to go to it
    def move_to_closest_obstacle(self,range,angle):

        #check which direction to turn
        if angle < 180:
            angular_vel = 1
        else:
            angular_vel = -1

        if not(angle >= 350 or angle <= 10):
            linear_vel = 0
            self.move_motor(linear_vel, angular_vel)
            self.move_motor(linear_vel, angular_vel)  
        else:
            linear_vel = 1
            angular_vel = 0
            self.move_motor(linear_vel, angular_vel)
            self.move_motor(linear_vel, angular_vel)

    def move_motor(self,fwd,ang):
        pub = rospy.Publisher(u'cmd_vel',Twist,queue_size = 10)
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
        #print(len(data.ranges))
        minRange = data.ranges[0]
        minRangeAngle = 0 
        for i in xrange(0,len(data.ranges)):
            if data.ranges[i] < minRange:
                minRange = data.ranges[i]
                minRangeAngle = i
        print minRangeAngle
        print minRange
        
        armLength = 0.3 #approximate
        if minRange > armLength:
            self.move_to_closest_obstacle(minRange, minRangeAngle)

# when started as a script
if __name__ == u'__main__':

    # start up a new node
    rospy.init_node(u'proximity',anonymous=True)

    # initialize behaviour
    proximity = ProximityLDS()

    # update at 10 hz untill shutdown
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        rate.sleep()

  
