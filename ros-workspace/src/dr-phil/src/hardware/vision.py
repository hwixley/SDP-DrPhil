#!/usr/bin/env python3
import rospy
from vision.camera import Camera
from vision.ray import Ray
import numpy as np
from sensor_msgs.msg import CameraInfo
import tf

class Vision:
    """ Node for processing camera feed and performing detections """
    def __init__(self):
        pass
        
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        self.camera = None

    def camera_info_callback(self,data):

        self.camera = Camera(data,"/base_link")        

        # get rid of subscriber
        self.camera_info_sub.unregister()


    
if __name__ == "__main__":
    rospy.init_node("vision",anonymous=True)
    vision = Vision()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()