#! /usr/bin/env python3

import rospy
import yaml
from sensor_msgs.msg import LaserScan
from os import path 

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing scan data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        scan_data = yaml.load(file_handle,Loader=yaml.FullLoader)
    # Parse
    msg = LaserScan()
    msg.range_max = scan_data["range_max"]
    msg.range_min = scan_data["range_min"]
    msg.scan_time = scan_data["scan_time"]

    msg.angle_increment = scan_data["angle_increment"]
    msg.time_increment = scan_data["time_increment"]

    msg.angle_min = scan_data["angle_min"] 
    msg.angle_max = scan_data["angle_max"]

    msg.intensities = [float(x) for x in scan_data["intensities"]]
    msg.ranges = [float(x) for x in scan_data["ranges"]]
    return msg

if __name__ == "__main__":
    
    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(path.join(path.dirname(path.realpath(__file__)),"scan.yml"))
    # append frame id
    camera_info_msg.header.frame_id = "base_scan"

    # Initialize publisher node
    rospy.init_node("fake_scan_pub", anonymous=True)
    publisher = rospy.Publisher("/scan_filtered", LaserScan, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()