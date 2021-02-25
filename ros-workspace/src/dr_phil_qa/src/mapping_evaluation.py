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
from dr_phil_qa.srv import EvaluateMapsResponse,EvaluateMaps
import os
import threading
import yaml 
import copy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GRID_VALUES(enum.IntEnum):
    OCCUPIED = 100
    FREE = 0
    UNKNOWN = -1
    
    def is_free(x):
        return x == GRID_VALUES.FREE
    def is_occupied(x):
        return x == GRID_VALUES.OCCUPIED
    def is_unknown(self,x):
        return x == GRID_VALUES.UNKNOWN

class MapEvaluator():
    """ node which compares a map message against another taken as the ground truth,
    and performs quantative measurements of mapping quality """


    def __init__(self,measured_topic,truth_topic,dump_scan,scan_param_namespace,dump_robot_velocity,odom_topic,store_explore_lite,explore_lite_namespace,run_periodically):
        
        self.measured_topic = measured_topic
        self.truth_topic = truth_topic

        self.tp_pub = rospy.Publisher("~tp_map",OccupancyGrid,queue_size=1)
        self.fp_pub = rospy.Publisher("~fp_map",OccupancyGrid,queue_size=1)
        self.tn_pub = rospy.Publisher("~tn_map",OccupancyGrid,queue_size=1)
        self.fn_pub = rospy.Publisher("~fn_map",OccupancyGrid,queue_size=1)
        self.combined_pub = rospy.Publisher("~combined_map",OccupancyGrid,queue_size=1)

        self.m_sub = rospy.Subscriber(self.measured_topic,OccupancyGrid,self.m_callback)
        self.GT_sub = rospy.Subscriber(self.truth_topic,OccupancyGrid,self.GT_callback)

        self.tp_map = None
        self.fp_map = None
        self.tn_map = None
        self.fn_map = None

        self.data_home_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..","data")

        self.evaluate_trigger = rospy.Service("~evaluate",EvaluateMaps,self.evaluate)
        self.data_guard = threading.Lock()
        
        self.measured_map = None
        self.truth_map = None
        self.values = {}

        self.dump_scan = dump_scan
        self.scan_param_namespace = scan_param_namespace

        self.run_periodically = run_periodically
        self.last_args = None 
        self.time_start = rospy.rostime.get_time()

        self.dump_twist = dump_robot_velocity
        if dump_robot_velocity:
            self.odom_sub = rospy.Subscriber(odom_topic,Odometry,self.odom_callback)
        else:
            self.odom_sub = None

        self.linear_vels = []
        self.ang_vels = []
        self.odom = None 

        self.store_explore_lite = store_explore_lite
        self.explore_lite_namespace = explore_lite_namespace

    def odom_callback(self,data : Odometry):
        self.odom = data

    def m_callback(self,data):
        self.measured_map = data

    def GT_callback(self,data):
        self.truth_map = data 

    def spin(self):
        if self.run_periodically and not ((self.measured_map is None) or (self.truth_map is None) or (self.last_args is None)):
            self.evaluate(self.last_args)
        
        self.visualise_evaluation()
        self.publish_values()



    def publish_values(self):
        #TODO: add publishers for values. keys
        pass

    def visualise_evaluation(self):
        combined_map = copy.copy(self.tp_map)

        for (p,m) in [(self.tp_pub,self.tp_map),
                    (self.fp_pub,self.fp_map),
                    (self.tn_pub,self.tn_map),
                    (self.fn_pub,self.fn_map)]:
            if m is not None:
                p.publish(m)
                
                # take average 
                combined_map.data = [ x if y == GRID_VALUES.UNKNOWN else 
                                        (y if x == GRID_VALUES.UNKNOWN else 
                                            GRID_VALUES.UNKNOWN )for x,y in zip(combined_map.data,m.data)]

        if combined_map is not None:
            self.combined_pub.publish(combined_map)


    def evaluate(self,args):
        """ performs the evaluation between ground truth and measured maps """

        self.last_args = args

        ## await map data first
        raw = {}
        rates = {}
        areas = {}
        quality = {}
        metrics = {}
        values = {}

        ## Areas - translation and rotation invariant

        with self.data_guard: # don't want maps to change mid calculation
            free_area_measured = self.calculate_area(self.measured_map)
            occupied_area_measured = self.calculate_area(self.measured_map,
                                                            include_condition=GRID_VALUES.is_occupied)

            free_area_truth = self.calculate_area(self.truth_map)
            occupied_area_truth = self.calculate_area(self.truth_map,
                                                            include_condition=GRID_VALUES.is_occupied)

            areas["free_area_mesured_over_truth"] = free_area_measured / free_area_truth
            areas["occupied_area_mesured_over_truth"] = occupied_area_measured / occupied_area_truth
            
            areas["total_area_mesured_over_truth"] = (occupied_area_measured + free_area_measured) / (occupied_area_truth + free_area_truth)

            ## Rates - not invariant 
            (tp_num,tp_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                            self.true_positive_counter,
                                                            self.nonzero_to_green_filter)

            (tn_num,tn_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                            self.true_negative_counter,
                                                            self.nonzero_to_yellow_filter)

            (fp_num,fp_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                            self.false_positive_counter,
                                                            self.nonzero_to_red_filter)

            (fn_num,fn_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                            self.false_negative_counter,
                                                            self.nonzero_to_blue_filter)

            if self.dump_twist:
                rospy.logerr(str(self.dump_twist))
                self.linear_vels.append(abs(self.odom.twist.twist.linear.x))
                self.ang_vels.append(abs(self.odom.twist.twist.angular.z))

                quality["average_ang_vel"] = sum(self.ang_vels)/ len(self.ang_vels) 
                quality["average_lin_vel"] = sum(self.linear_vels)/ len(self.linear_vels) 

        
        quality["time_elapsed"] = (rospy.rostime.get_time() - self.time_start) / 60

        raw["true_positives"] = tp_num
        raw["true_negatives"] = tn_num
        raw["false_positives"] = fp_num
        raw["false_negatives"] = fn_num

        rates["true_positive_rate"] = tp_num / (tp_num + fn_num)
        rates["false_positive_rate"] = fp_num / (fp_num + tn_num)
        rates["overall_error"] = (fp_num + fn_num) / (tp_num + fn_num + fp_num + tn_num)

        metrics["rates"] = rates
        metrics["raw"] = raw
        metrics["areas"] = areas 
        metrics["quality"] = quality
        values["values"] = metrics

        # save data for publishing too
        self.values = values

        # save maps for visualisation
        self.tp_map = tp_map
        self.tn_map = tn_map
        self.fp_map = fp_map
        self.fn_map = fn_map

        
        file_path = os.path.join(self.data_home_path,args.filename + ".yml")

        if os.path.isfile(file_path) and ((not self.run_periodically) or self.run_periodically and self.last_args == None):
            rospy.logerr("file already exists at: {0}. Aborting".format(file_path))
            return None

        with open(file_path, 'w') as f:  # You will need 'wb' mode in Python 2.x
            yaml.dump(values,f,default_flow_style=False)

        if self.dump_scan:
            scan_params = rospy.get_param(self.scan_param_namespace)
            if scan_params is not None:
                with open(os.path.join(os.path.dirname(file_path),args.filename+"-scan.yml"), 'w') as f:
                    yaml.dump(scan_params,f,default_flow_style=False)

        if self.store_explore_lite:
            explore_params = rospy.get_param(self.explore_lite_namespace)
            if explore_params is not None:
                with open(os.path.join(os.path.dirname(file_path),args.filename+"-explore.yml"), 'w') as f:
                    yaml.dump(explore_params,f,default_flow_style=False)
                    
        return EvaluateMapsResponse()

    def true_positive_counter(self,m_val,GT_val):
        return 1 if m_val == GT_val == GRID_VALUES.OCCUPIED else 0

    def true_negative_counter(self,m_val,GT_val):
        return 1 if m_val == GT_val == GRID_VALUES.FREE else 0

    def false_positive_counter(self,m_val,GT_val):
        return 1 if GT_val != GRID_VALUES.OCCUPIED and m_val == GRID_VALUES.OCCUPIED else 0
    
    def false_negative_counter(self,m_val,GT_val):
        return 1 if GT_val != GRID_VALUES.FREE and m_val == GRID_VALUES.FREE else 0
    
    def nonzero_to_red_filter(self,x):
        """ filter which shows non zero counts on the map """
        return 98 if x != 0 else GRID_VALUES.UNKNOWN
    
    def nonzero_to_green_filter(self,x):
        """ filter which shows non zero counts on the map """
        return 101 if x != 0 else GRID_VALUES.UNKNOWN
    
    def nonzero_to_blue_filter(self,x):
        """ filter which shows non zero counts on the map """
        return 99 if x != 0 else GRID_VALUES.UNKNOWN
    
    def nonzero_to_yellow_filter(self,x):
        """ filter which shows non zero counts on the map """
        return -20 if x != 0 else GRID_VALUES.UNKNOWN

    def apply_count_over_map(self,m : OccupancyGrid,GT : OccupancyGrid,counter : Callable[[Number,Number],Number], map_filter : Callable[[Number],Number] = lambda x: x):
        """ sweeps the counter function over each pair of matching cells in m and GT maps in this order
         and sums the output of counter over the whole map, argument of NAN is passed on values which do not exist on one of the maps.
         
         Returns tuple with the returned sum as well as union occupancy map containing the counter values at each pixel (rounded and clamped to uint8)
         
            Args:
                m: the measured occupancy map
                GT: the ground truth occupancy map
                counter: the counter method applied to each two corresponding points given as a tuple (m,GT), output of this function is summed, m or GT may be NaN when the map has no corresponding value
                map_filter: the function applied to the counter method output before populating the union_map (values are rounded)
         """

        # TODO: match different origin maps (probably not necessary)
        if m.info.origin != GT.info.origin:
            raise ValueError("Maps do not have the same origin")
        elif m.info.resolution != GT.info.resolution:
            raise ValueError("Maps do not have the same resolution")

        # assuming maps from here on have the same origin at point (0,0) and the same resolution
        # we can begin matching
        
        # first loop over values of GT, any leftover are looped after 
        count = 0.0

        # generate union map for visualisation
        union_map = OccupancyGrid()
        union_map.info.height = max(m.info.height,GT.info.height)
        union_map.info.width = max(m.info.width,GT.info.width)
        union_map.info.resolution = m.info.resolution
        union_map.info.origin = m.info.origin
        union_map.header = GT.header
        union_map.data = [-1] * union_map.info.height * union_map.info.width
        for y in range(GT.info.height):
            for x in range(GT.info.width):

                # element on GT map
                GT_elem = GT.data[self.map_index_from_2d(y,x,GT.info.width)]

                # corresponding element on m map or NaN if out of bounds 
                m_elem = numeric.NaN

                if y < m.info.height and x < m.info.width:
                    m_elem = m.data[self.map_index_from_2d(y,x,m.info.width)]
                    
                val = counter(m_elem,GT_elem)
                count += val
                union_map.data[self.map_index_from_2d(y,x,union_map.info.width)] = int(map_filter(val)) 

        # we then loop over the values of m which were not hit yet
        # we 2 regions to check, with possibility of each one not needing to be checked
        
        if GT.info.height < m.info.height:
            # loop over the region above GT over the full width of m
            for y in range(GT.info.height,m.info.height):
                for x in range(m.info.width):

                    # element on m map 
                    m_elem = m.data[self.map_index_from_2d(y,x,m.info.width)]

                    # GT elem does not exist by definition in this part
                    val = counter(m_elem,numeric.NaN)
                    count += val
                    union_map.data[self.map_index_from_2d(y,x,union_map.info.width)] =  int(map_filter(val))  

        if GT.info.width < m.info.width:
            # loop over the region to the right of GT over the full height of m
            for y in range(m.info.height):
                for x in range(GT.info.width,m.info.width):
                    # element on m map 
                    m_elem = m.data[self.map_index_from_2d(y,x,m.info.width)]

                    # GT elem does not exist by definition in this part
                    val = counter(m_elem,numeric.NaN)
                    count += val
                    union_map.data[self.map_index_from_2d(y,x,union_map.info.width)] =  int(map_filter(val)) 

        return (count,union_map)

    def map_index_from_2d(self,y,x,width):
        return y * width + x

    def calculate_area(self,map : OccupancyGrid,include_condition : Callable[[int],bool] = lambda x: x == GRID_VALUES.FREE):
        """ calculates total number of cells with prior satisfying the include_condition """
        area = sum([1 for x in map.data if include_condition(x)])
        
        return float(area)


if __name__ == "__main__":
    rospy.init_node("map_evaluator")

    measured_topic = rospy.get_param("~measured_topic","/map_measured")
    truth_topic = rospy.get_param("~truth_topic","/map_truth")
    dump_scan = rospy.get_param("~store_scan_params",False)
    scan_param_namespace = rospy.get_param("~scan_param_namespace","/slam_gmapping")
    run_periodically = rospy.get_param("~run_periodically",False)
    store_velocity = rospy.get_param("~store_robot_velocity",False)
    odom_topic = rospy.get_param("~odom_topic","/odom")
    store_explore_lite = rospy.get_param("~store_explore_lite",False)
    explore_lite_namespace = rospy.get_param("~explore_lite_namespace","/explore")

    rate = rospy.Rate(1)

    node = MapEvaluator(measured_topic,
                truth_topic,
                dump_scan,
                scan_param_namespace,
                store_velocity,
                odom_topic,
                store_explore_lite,
                explore_lite_namespace,
                run_periodically)

    if run_periodically:
        rospy.loginfo("run_periodically setting is on, after first service call will repeatedely evaluate")
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
