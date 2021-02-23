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


    def __init__(self,measured_map,truth_map):
        
        self.measured_map = measured_map
        self.truth_map = truth_map

        self.tp_pub = rospy.Publisher("~tp_map",OccupancyGrid,queue_size=10)
        self.fp_pub = rospy.Publisher("~fp_map",OccupancyGrid,queue_size=10)
        self.tn_pub = rospy.Publisher("~tn_map",OccupancyGrid,queue_size=10)
        self.fn_pub = rospy.Publisher("~fn_map",OccupancyGrid,queue_size=10)

        self.tp_map = None
        self.fp_map = None
        self.tn_map = None
        self.fn_map = None

    def visualise_evaluation(self):
        for (p,m) in [(self.tp_pub,self.tp_map),
                    (self.fp_pub,self.fp_map),
                    (self.tn_pub,self.tn_map),
                    (self.fn_pub,self.fn_map)]:
            if m is not None:
                p.publish(self.binarize_counter_map(m))
    
    def binarize_counter_map(self,map : OccupancyGrid):
        """ counter passed maps are not entirely fit for viewing, binarize them - all values between 0 and 99  are converted to FREE values, overflow is turned to OCCUPIED and underflow to UNKNOWN"""
        
        def classify(x):
            if x < 0:
                return GRID_VALUES.UNKNOWN
            elif x < 100:
                return GRID_VALUES.FREE
            else:
                return GRID_VALUES.OCCUPIED

        map.data = [classify(x) for x in map.data]
        return map

    def evaluate(self,file_path):

        values = {}

        """ called after both maps have been received and stored, performs the evaluation """


        ## Areas - translation and rotation invariant
        free_area_measured = self.calculate_area(self.measured_map)
        occupied_area_measured = self.calculate_area(self.measured_map,
                                                        include_condition=GRID_VALUES.is_occupied)

        free_area_truth = self.calculate_area(self.truth_map)
        occupied_area_truth = self.calculate_area(self.truth_map,
                                                        include_condition=GRID_VALUES.is_occupied)

        values["free_area_mesured_over_truth"] = free_area_measured / free_area_truth
        values["occupied_area_mesured_over_truth"] = occupied_area_measured / occupied_area_truth
        values["total_area_mesured_over_truth"] = (occupied_area_measured + free_area_measured) / (occupied_area_truth + free_area_truth)

        ## Rates - not invariant 
        (tp_num,tp_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                        self.true_positive_counter,
                                                        self.nonzero_to_red_filter)

        (tn_num,tn_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                        self.true_negative_counter,
                                                        self.nonzero_to_red_filter)

        (fp_num,fp_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                        self.false_positive_counter,
                                                        self.nonzero_to_red_filter)

        (fn_num,fn_map) = self.apply_count_over_map(self.measured_map,self.truth_map,
                                                        self.false_negative_counter,
                                                        self.nonzero_to_red_filter)

        values["true_positives"] = tp_num
        values["true_negatives"] = tn_num
        values["false_positives"] = fp_num
        values["false_negatives"] = fn_num

        # save maps for visualisation
        
        self.tp_map = tp_map
        self.tn_map = tn_map
        self.fp_map = fp_map
        self.fn_map = fn_map

        with open(file_path, 'w') as f:  # You will need 'wb' mode in Python 2.x
            w = csv.DictWriter(f, values.keys())
            w.writeheader()
            w.writerow(values)

    def true_positive_counter(self,m_val,GT_val):
        return 1 if m_val == GT_val == GRID_VALUES.OCCUPIED else 0

    def true_negative_counter(self,m_val,GT_val):
        return 1 if m_val == GT_val == GRID_VALUES.FREE else 0

    def false_positive_counter(self,m_val,GT_val):
        return 1 if m_val == GRID_VALUES.OCCUPIED and GT_val == GRID_VALUES.FREE else 0
    
    def false_negative_counter(self,m_val,GT_val):
        return 1 if m_val == GRID_VALUES.FREE and GT_val == GRID_VALUES.OCCUPIED else 0
    
    def nonzero_to_red_filter(self,x):
        """ filter which shows non zero counts on the map """
        return 254 if x != 0 else GRID_VALUES.UNKNOWN

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

    measured_topic = rospy.get_param("~measured","/map_measured")
    truth_topic = rospy.get_param("~truth","/map_truth")
    file_path = rospy.get_param("~file_path","data.csv")


    
    measured_map = rospy.wait_for_message(measured_topic,OccupancyGrid)
    truth_map = rospy.wait_for_message(truth_topic,OccupancyGrid)

    evaluator = MapEvaluator(measured_map,
                    truth_map)

    evaluator.evaluate(file_path)
    
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        evaluator.visualise_evaluation()
        rate.sleep()
