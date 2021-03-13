#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def rotate360():
    #Starts a new node
    rospy.init_node('map_explore', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Rotating the robot by 360deg")
    # larger than 360 because rospy.Time.now() has a slight delay
    angle = 490 #angle in deg
    speed = 40 #speed in deg/sec
    clockwise = True   

    #Converting from angles to radians
    relative_angle = np.deg2rad(angle)
    angular_speed = np.deg2rad(speed)

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        rotate360()
    except rospy.ROSInterruptException:
        pass