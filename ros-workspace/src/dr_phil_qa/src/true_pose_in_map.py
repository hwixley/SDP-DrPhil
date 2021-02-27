#!/usr/bin/env python3

from numpy.core import numeric
import rospy
from nav_msgs.msg import OccupancyGrid
import sys
import operator
import enum
import numpy as np
from typing import Callable
from numbers import Number
import csv
from dr_phil_qa.srv import FileNameResponse,FileName
import os
import threading
import yaml 
import copy 
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, PoseStamped, Point
import message_filters

class TruePoseParser():
    def __init__(self,true_odom_topic,map_topic,pose_topic):

        self.map_topic = map_topic

        self.pose_pub = rospy.Publisher(pose_topic,PoseStamped,queue_size=10)

        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, callback=self.map_callback)
        self.odom_sub = rospy.Subscriber(true_odom_topic, Odometry, callback=self.odom_callback)

        self.recent_map = None
        self.recent_pose = None

    def map_callback(self,map : OccupancyGrid):
        self.recent_map = map

    def odom_callback(self,odom : Odometry):
        if self.recent_map is None:
            rospy.logwarn("No map received on {}, trying again..".format(self.map_topic)) 
            return

        map_origin_pose = self.recent_map.info.origin

        # new_pose = Pose()
        # x,y,z = Point()
        # x = map_origin_pose.position.x + odom_pose.position.x
        # y = map_origin_pose.po
        # new_pose.position = map_origin_pose.position + odom_pose.position.
        # new_pose.orientation = odom_pose.orientation 
        self.recent_pose = PoseStamped()
        self.recent_pose.header = odom.header
        self.recent_pose.pose = odom.pose.pose

    def spin(self):
        if self.recent_pose is not None:
            self.pose_pub.publish(self.recent_pose)

if __name__ == "__main__":
    rospy.init_node("map_evaluator")

    true_odom_topic = rospy.get_param("~true_odom_topic","/gt/odom")
    map_topic = rospy.get_param("~map_topic","/map")
    pose_topic = rospy.get_param("~pose_topic","/gt/pose")
    rate = rospy.Rate(10)

    node = TruePoseParser(true_odom_topic,
                map_topic,
                pose_topic)

    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()