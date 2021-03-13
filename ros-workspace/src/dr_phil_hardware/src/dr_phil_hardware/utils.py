#!/usr/bin/env python3

from graphviz import Source
import rospy
import sys
from std_msgs.msg import String
import py_trees
import numpy as np
import tf.transformations as t



def quaternion_from_dirs(forward_x_axis,up_z_axis):
    assert(forward_x_axis.shape == (3,1) and up_z_axis.shape == (3,1))
    
    side_y_axis = np.cross(forward_x_axis[:,0],up_z_axis[:,0])[:,None]

    forward_x_axis = np.append(forward_x_axis/np.linalg.norm(forward_x_axis),
        np.array([[0]]),axis=0)
    up_z_axis = np.append(up_z_axis/np.linalg.norm(up_z_axis),
        np.array([[0]]),axis=0)


    side_y_axis = side_y_axis/ np.linalg.norm(side_y_axis)
    side_y_axis = np.append(side_y_axis,np.array([[0]]),axis=0)

    rot_mat = np.concatenate((forward_x_axis,side_y_axis,up_z_axis,np.array([[0],[0],[0],[1]])),axis=1)
    rospy.logerr(rot_mat)
    quat = t.quaternion_from_matrix(rot_mat)
    return quat

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

