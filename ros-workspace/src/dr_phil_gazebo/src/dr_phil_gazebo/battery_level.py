#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState
import time

class batterySim():

    def __init__(self):
        #time until low battery (11v) in mins
        self.low_battery_time = 30
        
        self.battery_pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)
        self.rate = rospy.Rate(10) 
        self.state = BatteryState()
        self.state.design_capacity = 1.79999995232
        self.state.voltage = 12.1
        self.state.percentage = self.state.voltage / 11.1
    
    def discharge(self):
        
        while not rospy.is_shutdown():

            self.state.voltage= self.state.voltage - (1.1 / (self.low_battery_time * 600))
            self.state.percentage = self.state.voltage / 11.1
            self.battery_pub.publish(self.state)
            self.rate.sleep()
            
            #turtlebot cannot operate below 10v
            if (self.state.voltage <= 10.0):
                rospy.logwarn("Battery too low! Operation at below 10v can lead to battery damage.")
  

if __name__ == '__main__':
    rospy.init_node('battery_level', anonymous=True)
    battery = batterySim()
    try:
        battery.discharge()
    except rospy.ROSInterruptException:
        pass
