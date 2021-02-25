#!/usr/bin/env python

from __future__ import division
from __future__ import absolute_import
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import sys 
class px100JointControllerMerger(object):
    def __init__(self):    

        # input, joints in order but first element is zero
        # based on: http://www.inf.ed.ac.uk/teaching/courses/sdp/SDP2020/turtlebot3_docs.pdf
        self.input_sub = rospy.Subscriber(u'joint_trajectory_point',Float64MultiArray,self.merge_callback) 

        # output, splits input to those publishers in order
        self.waist_pub = rospy.Publisher(u"/waist_controller/command",Float64,queue_size=10)
        self.shoulder_pub = rospy.Publisher(u"/shoulder_controller/command",Float64,queue_size=10)
        self.elbow_pub = rospy.Publisher(u"/elbow_controller/command",Float64,queue_size=10)
        self.wrist_pub = rospy.Publisher(u"/wrist_angle_controller/command",Float64,queue_size=10)
        self.fingerL_pub = rospy.Publisher(u"/left_finger_controller/command",Float64,queue_size=10)
        self.fingerR_pub = rospy.Publisher(u"/right_finger_controller/command",Float64,queue_size=10)
        

        self.state = [0,0,0,0,0,0]
        self.gripper_in_open_pos = 1.57
        self.gripper_in_closed_pos = -1.1

        self.l_finger_out_open_pos = 0.037
        self.l_finger_out_closed_pos = 0.015

        self.r_finger_out_open_pos = -0.037
        self.r_finger_out_closed_pos = -0.015   


    def merge_callback(self,joint_trajectory_point):
        """ distributes single message containing all arm joint configs requested to the multiple simulation controllers 
            based on: http://www.inf.ed.ac.uk/teaching/courses/sdp/SDP2020/turtlebot3_docs.pdf
        """
        self.state = joint_trajectory_point.data
        self.propagate_state()
    

    def interpolate(self,min_v,max_v,r):
        """ returns v which is r of the way between min_v and max_v (0 > r < 1) 
            min_v is < max_v 
        """

        # if min_v > max_v:
        #     min_v,max_v = max_v,min_v
        total = max_v - min_v
        return (r * total) + min_v

    def propagate_state(self):
        """ distributes state to each controller based on self.state """

        # first 4 need no changes
        self.waist_pub.publish(self.state[1])
        self.shoulder_pub.publish(self.state[2])
        self.elbow_pub.publish(self.state[3])
        self.wrist_pub.publish(self.state[4])
        
        # input only gives one value for gripper, interpolate both fingers
        #           a         b
        # closed --------in-------> open
        # open ratio = a/(a+b)
        a = abs(self.state[5] - self.gripper_in_closed_pos) 
        total = (abs(self.gripper_in_open_pos) + abs(self.gripper_in_closed_pos))
        open_ratio = a / total
        # interpolate left finger motion
        l_finger_pos = self.interpolate(self.l_finger_out_closed_pos,
                                    self.l_finger_out_open_pos,
                                    open_ratio)
        
        self.fingerL_pub.publish(l_finger_pos)
        # right finger's range of motion is exactly opposite to that of l
        self.fingerR_pub.publish(-l_finger_pos)



# when started as a script
if __name__ == u'__main__':

    # start up a new node
    rospy.init_node(u'px100_joint_controller_merger',anonymous=True)

    # initialize behaviour
    merger = px100JointControllerMerger()

    # update at 10 hz untill shutdown
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except (ROSException, KeyboardInterrupt):
            sys.exit(0)
