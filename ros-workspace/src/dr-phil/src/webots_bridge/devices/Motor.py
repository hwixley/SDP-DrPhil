from devices.Device import Device
from webots_ros.srv import set_int,set_float,lidar_get_info,lidar_get_layer_range_image
import rospy 

class RotationalMotor(Device):
    def __init__(self,robot_unique_name,device_name):
        super().__init__(robot_unique_name,device_name)

        self.set_position_srv = rospy.ServiceProxy(self.get_service_name("set_position"),set_float)
        self.set_velocity_srv = rospy.ServiceProxy(self.get_service_name("set_velocity"),set_float)

        self.vel = 0
        self.pos = 0

        self.last_vel = 0
        self.last_pos = 0

    def set_velocity(self,vel):
        self.vel = vel

    def set_position(self,pos):
        self.pos = pos 

    def update_device(self,dt):
        if self.last_vel != self.vel:
            self.set_velocity_srv(self.vel)
            self.last_vel = self.vel

        if self.last_pos != self.pos:
            self.set_position_srv(self.pos)
            self.last_pos = self.pos

    def setup_device(self,frequency=15):
        pass
        
        
