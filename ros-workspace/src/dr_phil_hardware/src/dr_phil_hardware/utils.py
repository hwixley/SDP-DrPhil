#!/usr/bin/env python3

from graphviz import Source
import rospy
import sys
from std_msgs.msg import String
import py_trees
from geometry_msgs.msg import Pose
from dr_phil_hardware.vision.utils import quat_from_yaw
import math
import numpy as np
import tf.transformations as t
import copy 




def quat_to_vec(quat : list):
    """returns the vector resulting from rotating the x unit vector by the given quaternion

    Args:
        quat (list): [description]

    Returns:
        [type]: [description]
    """

    (a,b,c) = t.euler_from_quaternion(quat)
    mat = t.euler_matrix(a,b,c)
    vec =  mat[:,:-1] @ np.array([[1],[0],[0]])

    return vec


            

def rotate_pose_by_yaw(yaw,pose : Pose):
    """Returns new pose with orientation rotated by the given yaw angle in radians

    Args:
        yaw ([type]): [description]
        pose (Pose): [description]

    Returns:
        [type]: [description]
    """
    curr_quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    (a,b,c) = t.euler_from_quaternion(curr_quat)
    rot = quat_from_yaw(yaw)
    new_quat = t.quaternion_multiply(curr_quat,rot)

    new_pose = Pose()
    new_pose.position = copy.deepcopy(pose.position)
    new_pose.orientation.x,new_pose.orientation.y,new_pose.orientation.z,new_pose.orientation.w = new_quat

    return  new_pose


def interpolate(self,min_v,max_v,r):
    """ returns v which is r of the way between min_v and max_v (0 > r < 1) 
        min_v is < max_v 
    """

    # if min_v > max_v:
    #     min_v,max_v = max_v,min_v
    total = max_v - min_v
    return (r * total) + min_v

## Utility script used to visualise controller tree
## requires graphviz python package to be installed
def visualise_tree():
    received = False

    def receive(data):
        received = True
        print(data.data)
        src=  Source(data.data)
        src.render('controller-tree.gv',view=True)

    rospy.init_node("vis",anonymous=True)

    # find topic containing dot tree and listen to one of the 
    # first message then interpret it and shutdown
    topic = ""
    for t in rospy.get_published_topics():
        if "/dot/tree" in t[0]:
            print(t)
            topic = t[0]

    rospy.Subscriber(topic,String,receive)

    rate = rospy.Rate(10)
    while not received and not rospy.is_shutdown():
        rate.sleep()

