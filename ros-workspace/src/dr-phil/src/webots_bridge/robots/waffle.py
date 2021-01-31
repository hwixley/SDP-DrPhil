
from devices.Lidar import Lidar 
from devices.Motor import RotationalMotor
from geometry_msgs.msg import Twist
import rospy

class Waffle:
    
    def __init__(self,r_name):
        self.r_name = r_name
        self.lidar = Lidar(r_name)
        self.r_motor = RotationalMotor(r_name,"right_wheel_motor")
        self.l_motor = RotationalMotor(r_name,"left_wheel_motor")

        self.cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,self.cmd_vel_callback)

        self.max_vel = 6


    def setup(self):
        self.lidar.setup_device()
        self.r_motor.setup_device()
        self.l_motor.setup_device()
        
    def update(self,dt):
        self.r_motor.update_device(dt)
        self.l_motor.update_device(dt)
        self.lidar.update_device(dt)

    def cmd_vel_callback(self,twist):
        fwd = twist.linear.x
        angular = twist.angular.z

        # TODO: this is a very simplified differential controller
        if angular == 0:
            self.set_forward_vel(fwd)
        elif fwd == 0:
            self.set_turn_vel(angular,0)
        else:
            self.set_turn_vel(angular,fwd/self.max_vel)

    def set_forward_vel(self,vel):
        self.r_motor.set_position(float("inf"))
        self.l_motor.set_position(float("inf"))
        self.r_motor.set_velocity(float(vel))
        self.l_motor.set_velocity(float(vel))

    def set_turn_vel(self,vel,throttle):
        self.r_motor.set_position(float("inf"))
        self.l_motor.set_position(float("inf"))

        if vel < 0:
            self.r_motor.set_velocity(float(-vel))
            self.l_motor.set_velocity(float(min(throttle*-vel,-vel)))
        else: 
            self.r_motor.set_velocity(float(min(throttle*vel,vel)))
            self.l_motor.set_velocity(float(vel))
