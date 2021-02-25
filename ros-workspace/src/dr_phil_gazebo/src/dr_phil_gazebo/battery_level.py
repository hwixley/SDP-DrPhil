#!/usr/bin/env python
from __future__ import division
from __future__ import absolute_import
import rospy
from sensor_msgs.msg import BatteryState
import time

class batterySim(object):

    def __init__(self):
        #time until low battery (11v) in mins
        self.low_battery_time = 30
        
        self.battery_pub = rospy.Publisher(u'battery_state', BatteryState, queue_size=10)
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
                raise Exception(u"Battery too low! Operation at below 10v can lead to battery damage.")
  

if __name__ == u'__main__':
    rospy.init_node(u'battery_level', anonymous=True)
    battery = batterySim()
    try:
        battery.discharge()
    except rospy.ROSInterruptException:
        pass
