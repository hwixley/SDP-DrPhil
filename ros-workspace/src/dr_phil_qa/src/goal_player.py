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
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from tf.transformations import quaternion_from_euler

class GoalPlayer():
    """ plays back a move base trajectory from yml file as soon as localization_evaluation.py starts up, and calls it's stop service when it's finished """
    def __init__(self,trajectory_yml,halt_service,map_frame,loop_count):

        self.trajectory_yml = trajectory_yml
        self.pose_list = []
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.halt_service = halt_service
        self.halt_proxy  = rospy.ServiceProxy(self.halt_service,Empty)
        self.map_frame = map_frame
        self.loop_count = loop_count

        self.load_trajectory(trajectory_file)

    def load_trajectory(self,path):

        data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..","data","trajectories",path + ".yml")

        with open(data_path,'r') as f:
            list_point = yaml.full_load(f)
            
            for p in list_point:
                point = p["point"]

                pose = PoseStamped()
                pose.pose.position.x = point["position"]["x"]
                pose.pose.position.y = point["position"]["y"]

                orientation = quaternion_from_euler(0,0, point["orientation"]["z"])
                
                pose.pose.orientation.x = orientation[0]
                pose.pose.orientation.y = orientation[1]
                pose.pose.orientation.z = orientation[2]
                pose.pose.orientation.w = orientation[3]
 
                self.pose_list.append(pose)

    def play_trajectory(self):

        
        i = 0
        for pose in self.pose_list:
            goal = MoveBaseGoal()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = rospy.Time.now()
            goal.target_pose = pose
            rospy.loginfo("new pose target in frame {0}: {1}".format(str(self.map_frame),str(goal.target_pose)))


            self.client.send_goal_and_wait(goal)
            rospy.loginfo("reached goal {0}".format(i))
            i+=1

        rospy.loginfo("played whole trajectory")
        self.halt_proxy.call()
        self.loop_count -= 1
        
        if self.loop_count > 0:
            self.loop_count -= 1
            rospy.loginfo("repeating loop, loops left: {0}".format(self.loop_count))
            self.play_trajectory()
            
        

if __name__ == "__main__":
    rospy.init_node("map_evaluator",log_level=rospy.INFO)

    trajectory_file = rospy.get_param("~trajectory_file","trajectory.yml")
    localization_evaluator_topic_trigger = rospy.get_param("~localization_evaluator_pose_topic","/true_path")
    localization_evaluator_service_to_call = rospy.get_param("~localization_evaluator_service_stop","/localization_evaluator/stop")
    loop_count = rospy.get_param("~loop_count",3)
    map_frame = rospy.get_param("~map_frame","map")

    rospy.wait_for_message(localization_evaluator_topic_trigger,Path)

    node = GoalPlayer(trajectory_file,localization_evaluator_service_to_call,map_frame,loop_count)

    node.play_trajectory()
