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
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, PoseStamped
import message_filters

class LocalizationEvaluator():
    def __init__(self,measured_topic,truth_topic):

        self.measured_topic = measured_topic
        self.truth_topic = truth_topic

        self.activated = False 
        self.was_activated = False
        self.start_trigger = rospy.Service("~start_evaluate",FileName,self.start)
        self.start_trigger = rospy.Service("~stop_evaluate",FileName,self.stop)

        self.true_pose_sub = message_filters.Subscriber(self.truth_topic,Pose)
        self.measured_pose_sub = message_filters.Subscriber(self.measured_topic,PoseWithCovariance)
        ts = message_filters.ApproximateTimeSynchronizer([self.true_pose_sub,self.measured_pose_sub])
        ts.registerCallback(self.pose_callback)


        self.vis_true_path_pub = rospy.Publisher("~/true_path",Path,queue_size=10)
        self.vis_measured_path_pub = rospy.Publisher("~/measured_path",Path,queue_size=10)

        self.path_measured = None
        self.path_true = None

    def pose_callback(self,pose_true,pose_measured):
        self.true_pose
        print(pose_true,pose_measured)

    def start(self):
        self.activated = True
        
    def stop(self):
        self.activated = False

    def visualise_evaluation(self):
        if self.path_true is not None:
            self.vis_true_path_pub.publish(self.path_true)
            
        if self.path_measured is not None:
            self.vis_measured_path_pub.publish(self.path_measured)


    def publish_values(self):
        pass

    def spin(self):
        if self.activated == True and self.true_pose is not None and self.measured_pose_cv is not None:
            self.visualise_evaluation()
            self.publish_values()

        elif self.was_activated == True and self.activated == False:
            # halt
            pass

        self.was_activated = self.activated

if __name__ == "__main__":
    rospy.init_node("map_evaluator")

    measured_topic = rospy.get_param("~measured_topic","/amcl_pose")
    truth_topic = rospy.get_param("~truth_topic","/true_pose")
    
    rate = rospy.Rate(1)

    node = LocalizationEvaluator(measured_topic,
                truth_topic)

    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
