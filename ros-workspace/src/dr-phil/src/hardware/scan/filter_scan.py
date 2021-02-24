#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class FilterScan():

    def __init__(self,source_topic="/scan",target_topic="/scan_filtered", min_range=0.16):
        """ 
            Args:
                source_topic: the topic from which to get laser scan data
                target_topic: the topic to which publish filtered scan data
                min_range: the range below which ranges are considered invalid
        """
        self.source = source_topic
        self.target = target_topic
        self.min_range = min_range
        self.sub = rospy.Subscriber(source_topic,LaserScan,callback=self.filter)
        self.pub = rospy.Publisher(target_topic,LaserScan,queue_size=10)

    def filter(self,laserScan):
        # values under range_min are discarded
        
        # set new min range
        laserScan.range_min = self.min_range

        # re-publish on target topic

        self.pub.publish(laserScan)


if __name__ == "__main__":
    rospy.init_node("filter_scan")
    source = rospy.get_param("~scan/source","/scan")
    target = rospy.get_param("~scan/target","/scan_filtered")
    min_range = rospy.get_param("~filter/min_range",0.16)

    rospy.loginfo("filtering from topic {0} to topic {1} with {2} range".format(source,target,min_range))
    node = FilterScan(source,target,min_range)

    # block the thread so process doesn't dy
    rospy.spin()
    print("shutdown")