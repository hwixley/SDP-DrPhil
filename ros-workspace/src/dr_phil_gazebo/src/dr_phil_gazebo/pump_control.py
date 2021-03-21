import rospy
from motors import Motors
from time import time, sleep
 from std_msgs.msg import String

#boolean variable for determing running or not
turnedOn = True
​
MOTOR_ID = 0
SPEED = 100

#function for turning motor on at a specific speed, specified above
​def pumprun(motor_id,speed):
	mc = Motors()
	motor_id = MOTOR_ID
	speed = SPEED
	mc.move_motor(motor_id, speed)
    
def callback(data):
​
    if data==1:
        turnedOn = True
        while turnedOn:
            print ('pump is on')
            pumprun(MOTOR_ID,SPEED)
            
    if data==0:
        turnedOn = False
​
def listener():
    rospy.init_node('pump_controller', anonymous=True)
    rospy.Subscriber('/spray_controller/command', Float32, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
​
if __name__ == '__main__':
    listener()

