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
    def __init__(self,map_origin_world):
        pass
    
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
