from devices.Device import Device
from webots_ros.srv import set_int,set_float,lidar_get_info,lidar_get_layer_range_image
import rospy 

class RotationalMotor(Device):
    def __init__(self,robot_unique_name,device_name):
        super().__init__(robot_unique_name,device_name)

        self.set_position_srv = rospy.ServiceProxy(self.get_service_name("set_position"),set_float)
        self.set_velocity_srv = rospy.ServiceProxy(self.get_service_name("set_velocity"),set_float)


    def set_velocity(self,vel):
        self.set_velocity_srv(vel)

    def set_position(self,pos):
        self.set_position_srv(pos)

    def update_device(self,dt):
        pass

    def setup_device(self,frequency=15):
        pass
        
        
