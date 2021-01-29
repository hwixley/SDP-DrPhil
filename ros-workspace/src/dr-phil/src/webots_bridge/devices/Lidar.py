#!/usr/bin/env python3

from devices.Device import Device
from devices.Motor import RotationalMotor
from webots_ros.srv import set_int,set_float,lidar_get_info,lidar_get_layer_range_image
import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import cv2
# from cv_bridge import CvBridge,CvBridgeError
# from cv_bridge.boost.cv_bridge_boost import getCvType

class Lidar(Device):
    def __init__(self,robot_unique_name):
        super().__init__(robot_unique_name,"LDS_01")

        self.enable_srv = rospy.ServiceProxy(self.get_service_name("enable"), set_int)
        self.get_info_srv = rospy.ServiceProxy(self.get_service_name("get_info"),lidar_get_info)
        self.get_layer_range_image_srv = rospy.ServiceProxy(self.get_service_name("get_layer_range_image"),lidar_get_layer_range_image)

        self.main_motor = RotationalMotor(robot_unique_name,"LDS_01_main_motor")
        self.secondary_motor = RotationalMotor(robot_unique_name,"LDS_01_secondary_motor")

        self.scan_pub = rospy.Publisher('scan',LaserScan,queue_size=10)

        self.min_range = 0 
        self.max_range = 0

        # self.bridge = CvBridge()

    def update_device(self,dt):
        # receive and interpret image
        img = self.get_layer_range_image_srv().image
        # print(img)
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        #     # TODO: fill in rest of fields according to http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        #     scan = LaserScan()
        #     scan.ranges = cv_image
        #     self.scan_pub.publish(scan)
        # except CvBridgeError as e:
        #     print(e)

        



        pass

    def setup_device(self,frequency=15):
        """ enables the lidar """
        self.enable_srv(frequency)
        self.main_motor.set_position(float("inf"))
        self.secondary_motor.set_position(float("inf"))

        self.main_motor.set_velocity(30.)
        self.secondary_motor.set_velocity(60.)

        lidar_info = self.get_info_srv()
        self.min_range = lidar_info.minRange
        self.max_range = lidar_info.maxRange

        
