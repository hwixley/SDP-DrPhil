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

    
    def move_motor(self,fwd,ang):
        pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
        mc = Twist()
        mc.linear.x = fwd
        mc.angular.z = ang
        pub.publish(mc)

    #TODO: Given an angle and position of nearest obstacle, instruct the robot to go to it
    def move_to_closest_obstacle(self,position,angle):
        pass


    def read_laser_scan_data(self):
        rospy.Subscriber('scan',LaserScan,laser_scan_callback)

    #Test to move it 
    def move_example(self):
         self.move_motor(forward_speed,turn_speed)


      
        forward_speed = 1    
        turn_speed = 1
        # start_time = time()
        # duration = 5 #in seconds
        # while time()<start_time+duration:
        #     try:
        #         read_laser_scan_data()
        #         move_motor(forward_speed,turn_speed)
        #     except rospy.ROSInterruptException:
        #         pass
        # else:
        #     move_motor(0,0)

    def callback(self,data):
        self.move_example()


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

  
