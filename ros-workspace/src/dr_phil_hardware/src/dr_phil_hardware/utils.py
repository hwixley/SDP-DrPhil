#!/usr/bin/env python3

from __future__ import absolute_import
from graphviz import Source
import rospy
import sys
from std_msgs.msg import String

## Utility script used to visualise controller tree
## requires graphviz python package to be installed
def visualise_tree():
    received = False


    def receive(data):
        received = True
        print data.data
        src=  Source(data.data)
        src.render(u'controller-tree.gv',view=True)

    rospy.init_node(u"vis",anonymous=True)

    # find topic containing dot tree and listen to one of the 
    # first message then interpret it and shutdown
    topic = u""
    for t in rospy.get_published_topics():
        if u"/dot/tree" in t[0]:
            print t
            topic = t[0]

    rospy.Subscriber(topic,String,receive)

    rate = rospy.Rate(10)
    while not received and not rospy.is_shutdown():
        rate.sleep()
