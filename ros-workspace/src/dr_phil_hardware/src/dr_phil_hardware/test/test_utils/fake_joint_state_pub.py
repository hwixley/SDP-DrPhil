#!/usr/bin/env python3

"""
Fake joint state publisher which just forwards joint_trajectory_point data and initially publishes all zeros

"""

import rospy 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,Empty



class FakeJointStatePub():

    def __init__(self) -> None:
        self.s = rospy.Subscriber("/joint_trajectory_point",Float64MultiArray,callback=self.cb)
        self.p = rospy.Publisher("/joint_states",JointState,queue_size=10)
        self.sub_reset = rospy.Subscriber("/reset_joints",Empty,self.reset_cb)
        self.sub_reset = rospy.Subscriber("/simulate_obstacle",Empty,self.simulate_cb)
        self.last_received = [0,0,0,0,0,0]
        self.fake_obstacle = False 
        self.frozen_vals = None

    def cb(self,point):
        self.last_received = list(point.data)

    def reset_cb(self,data):
        self.last_received = [0,0,0,0,0,0]

    def simulate_cb(self,data):
        self.fake_obstacle = not self.fake_obstacle
        self.frozen_vals = self.last_received

    def spin(self):
        j = JointState()
        j.header.stamp = rospy.Time.now() 
        j.name = ["joint1","joint2","joint3","joint4","gripper","gripper_spindle","gripper_sub"]

        j.position = [0] * len(j.name)
        j.velocity = [0] * len(j.name)
        j.effort = [0] * len(j.name)

        use_vals = self.last_received if not self.fake_obstacle else self.frozen_vals

        for i,n in enumerate(j.name[0:5]):
            j.position[i] = use_vals[i+1] # skipping first element
            j.effort[i] = 0 # ye
            j.velocity[i] = 0 # ye
            
            if i == 4: # gripper
                j.position[6] = -use_vals[i+1] # gripper sub
                j.effort[6] = 0
                j.velocity[6] = 0

        j.position[5] = 0
        j.effort[5] = 0
        j.effort[5] = 0

        self.p.publish(j)

if __name__ == "__main__":

    rospy.init_node("fake_jointstate_pub")

    n = FakeJointStatePub()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        n.spin()
        rate.sleep()