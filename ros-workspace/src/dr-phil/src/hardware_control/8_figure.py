#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import time

def move_motor(fwd,ang):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang
    pub.publish(mc)

if __name__ == '__main__':
    rospy.init_node('8_figure',anonymous=True)
    start_time = time()
    duration = 35 #in seconds
    move_motor(0,0)

    while time()<start_time+duration:
        try:
            timestep = 2
            speed_forward = 5
            speed_turn = 3.5
            once = False
            curr_time = time() - start_time
            # forward
            if curr_time < 1 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn right 90
            elif curr_time < 2 * timestep and not once:
                move_motor(0,-speed_turn)
                once = True
            # forward
            elif curr_time < 3 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn left 90
            elif curr_time < 4 * timestep and not once:
                move_motor(0,speed_turn)
                once = True
            # forward
            elif curr_time < 5 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn left 90
            elif curr_time < 6 * timestep and not once: 
                move_motor(0,speed_turn)
                once = True
            # forward
            elif curr_time < 7 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn left 90
            elif curr_time < 8 * timestep and not once:
                move_motor(0,speed_turn)
                once = True
            # forward 
            elif curr_time < 10 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn left 90
            elif curr_time < 11 * timestep and not once:
                move_motor(0,speed_turn)
                once = True
            # forward
            elif curr_time < 12 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn right 90
            elif curr_time < 13 * timestep and not once:
                move_motor(0,-speed_turn)
                once = True
            # forward
            elif curr_time < 14 * timestep and not once:
                move_motor(speed_forward,0)
                once = True
            # turn right 90
            elif curr_time < 15 * timestep and not once:
                move_motor(0,-speed_turn)
                once = True
            # forward
            elif curr_time < 16 * timestep and not once:
                move_motor(speed_forward,0)
                once = True

        except rospy.ROSInterruptException:
            move_motor(0,0)
    else:
        move_motor(0,0)
    move_motor(0,0)