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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseStamped, Point
from std_msgs.msg import Header
import message_filters
from std_srvs.srv import Empty,EmptyResponse

class LocalizationEvaluator():
    def __init__(self,measured_topic,truth_topic):

        self.measured_topic = measured_topic
        self.truth_topic = truth_topic

        self.activated = False 
        self.was_activated = False
        self.start_trigger = rospy.Service("~start_evaluate",FileName,self.start)
        self.stop_trigger = rospy.Service("~stop_evaluate",Empty,self.stop)

        self.true_pose_sub = message_filters.Subscriber(self.truth_topic,PoseStamped)
        self.measured_pose_sub = message_filters.Subscriber(self.measured_topic,PoseWithCovarianceStamped)

        t = message_filters.ApproximateTimeSynchronizer([self.true_pose_sub,self.measured_pose_sub],queue_size=10,slop=0.1)
        t.registerCallback(self.pose_callback)

        self.vis_true_path_pub = rospy.Publisher("~/true_path",Path,queue_size=10)
        self.vis_measured_path_pub = rospy.Publisher("~/measured_path",Path,queue_size=10)


        self.true_pose_recent = None
        self.measured_pose_recent = None

        self.path_measured = None
        self.path_true = None
        self.filename = None
        self.values = {}
        self.start_time = None

    def pose_callback(self,true,measured):
        self.true_pose_callback(true)
        self.measured_pose_callback(measured)

    def true_pose_callback(self,pose : PoseStamped):
        self.true_pose_recent = pose 

    def measured_pose_callback(self,pose : PoseWithCovarianceStamped):

        self.measured_pose_recent = PoseStamped()
        self.measured_pose_recent.header = pose.header
        self.measured_pose_recent.pose = pose.pose.pose

    def start(self,filename):
        self.activated = True
        self.filename = filename.filename
        self.path_measured = None 
        self.path_true = None

        self.start_time = rospy.get_time()

        return FileNameResponse()

    def stop(self,empty):
        self.activated = False


        if self.filename is None:
            rospy.logerr("You needto call start first!")
            return None

        data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..","data",self.filename+".yml")
        
        try:
            with open(data_path, 'w') as f:  # You will need 'wb' mode in Python 2.x
                yaml.dump(self.values,f,default_flow_style=False)
        except:
            rospy.logerr("Could not write trajectory file at: {0}".format(str(data_path)))
            return None

        return EmptyResponse()


    def point_to_np(self,point : Point):
        return np.array([point.x,point.y,point.z])


    def abs_pos_difference_counter(self,measured : Pose,true : Pose):
        a = self.point_to_np(true.position)
        b = self.point_to_np(measured.position)
        return np.linalg.norm(a - b)

    def apply_counter_over_paths(self,measured: Path, true :Path, counter:Callable[[Pose,Pose],Number]):
        """ applies counting function over the paths in corresponding points and sums the result """

        count = 0
        
        for (m,GT) in zip(measured.poses,true.poses):
            count += counter(m.pose,GT.pose)

        return count

    def calculate_path_length(self,p :Path):

        if len(p.poses) <= 1:
            return 0

        total = 0
        prev_pos = p.poses[0].pose.position

        for i in range(1,len(p.poses)):
            pose = p.poses[i]
            pos = pose.pose.position
            total += np.linalg.norm(self.point_to_np(pos) - self.point_to_np(prev_pos))

        return total

    def evaluate(self):
        if self.path_measured is None:
            self.path_measured = Path()
            self.path_measured.header = Header()
            self.path_measured.header.stamp = rospy.Time.now()
            self.path_measured.header.frame_id = self.measured_pose_recent.header.frame_id

        if self.path_true is None:
            rospy.logerr("reset path")
            self.path_true = Path()
            self.path_true.header = Header()
            self.path_true.header.stamp = rospy.Time.now()
            self.path_true.header.frame_id = self.measured_pose_recent.header.frame_id

        
        self.measured_pose_recent.header.stamp = rospy.Time.now()
        self.true_pose_recent.header.stamp = rospy.Time.now()

        self.path_measured.poses.append(self.measured_pose_recent)
        self.path_true.poses.append(self.true_pose_recent)

        metrics = {}
        raw = {}

        raw["absolute_error"] = float(self.apply_counter_over_paths(self.path_measured,self.path_true,self.abs_pos_difference_counter))
        raw["measurements_count"] = len(self.path_true.poses)
        raw["time_passed"] = (rospy.get_time() - self.start_time) / 60
        if len(self.path_true.poses) > 1 and len(self.path_measured.poses) > 1:

            raw["true_path_length"] = float(self.calculate_path_length(self.path_true))
            raw["measured_path_length"] = float(self.calculate_path_length(self.path_measured))

            if raw["true_path_length"] > 0:
                metrics["measured_over_true_path_length"] = raw["measured_path_length"] / raw["true_path_length"]
                metrics["abs_error_per_true_path_meter"] = raw["absolute_error"] / raw["true_path_length"]

        metrics["abs_error_per_measurement"] = raw["absolute_error"] / raw["measurements_count"]
        metrics["abs_error_per_minute"] = raw["absolute_error"] / raw["time_passed"]
        metrics["abs_error_per_minute"] = raw["absolute_error"] / raw["time_passed"]



        self.values["raw"] = raw
        self.values["metrics"] = metrics

    def publish_values(self):
        if self.path_true is not None:
            self.path_true.header.stamp = rospy.Time.now()
            self.vis_true_path_pub.publish(self.path_true)
            
        if self.path_measured is not None:
            self.path_measured.header.stamp = rospy.Time.now()
            self.vis_measured_path_pub.publish(self.path_measured)

    def spin(self):
        if self.activated == True and self.true_pose_recent is not None and self.measured_pose_recent is not None:
            self.evaluate()
            self.publish_values()

        elif self.was_activated == True and self.activated == False:
            # halt
            pass

        self.was_activated = self.activated

if __name__ == "__main__":
    rospy.init_node("localization_evaluator",log_level=rospy.INFO)

    measured_topic = rospy.get_param("~measured_topic","/amcl_pose")
    truth_topic = rospy.get_param("~truth_topic","/true_pose")
    
    rate = rospy.Rate(1)

    node = LocalizationEvaluator(measured_topic,
                truth_topic)

    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
