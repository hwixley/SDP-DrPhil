#!/usr/bin/env python3

from devices.Device import Device
from devices.Motor import RotationalMotor
from webots_ros.srv import set_int,set_float,lidar_get_info,lidar_get_layer_range_image,lidar_get_frequency_info
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge,CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import math
from std_msgs.msg import Header

class Lidar(Device):
    def __init__(self,robot_unique_name,device_name):
        super().__init__(robot_unique_name,device_name)

        self.enable_srv = rospy.ServiceProxy(self.get_service_name("enable"), set_int)
        self.get_info_srv = rospy.ServiceProxy(self.get_service_name("get_info"),lidar_get_info)
        self.get_freq_info_srv = rospy.ServiceProxy(self.get_service_name("get_frequency_info"),lidar_get_frequency_info)
        self.get_layer_range_image_srv = rospy.ServiceProxy(self.get_service_name("get_layer_range_image"),lidar_get_layer_range_image)

        self.main_motor = RotationalMotor(robot_unique_name,"LDS_01_main_motor")
        self.secondary_motor = RotationalMotor(robot_unique_name,"LDS_01_secondary_motor")

        self.scan_pub = rospy.Publisher('scan',LaserScan,queue_size=10)

        # TODO: Support for more layers
        self.scan_sub = rospy.Subscriber(self.get_service_name("laser_scan/layer0"),LaserScan,self.forward_laser_topic)
        

        self.min_range = 0 
        self.max_range = 0
        self.horizontal_resolution = 0
        self.fov = 0
        self.no_layers = 0
        self.frequency = 0
        self.sampling_period = 0

        self.bridge = CvBridge()



    def forward_laser_topic(self,laserScan):
        self.scan_pub.publish(laserScan)
        
    def update_device(self,time):
        # receive and interpret image
        self.update_frequency()

    # frequency might change, call on each update
    def update_frequency(self):
        self.frequency = self.get_freq_info_srv().frequency
    

    def setup_device(self,sampling_period=15):
        """ enables the lidar """
        self.sampling_period = sampling_period
        self.enable_srv(sampling_period)
        self.main_motor.set_position(float("inf"))
        self.secondary_motor.set_position(float("inf"))

        self.main_motor.set_velocity(30.)
        self.secondary_motor.set_velocity(60.)

        lidar_info = self.get_info_srv()

        self.min_range = lidar_info.minRange
        self.max_range = lidar_info.maxRange
        self.horizontal_resolution = lidar_info.horizontalResolution
        self.fov = lidar_info.fov
        self.no_layers = lidar_info.numberOfLayers

        self.update_frequency()

        
