#!/usr/bin/env python3

from graphviz import Source
import rospy
import sys
from std_msgs.msg import String
import py_trees



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

