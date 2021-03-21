import rospy
from std_msgs.msg import Float32
import time
from rpi_ws281x import Color
from grove.grove_ws2813_rgb_led_strip import GroveWS2813RgbStrip
​
​
# connect to pin 12(slot PWM)
PIN   = 18
# For Grove - WS2813 RGB LED Strip Waterproof - 30 LED/m
# there is 30 RGB LEDs.
COUNT = 30
strip = GroveWS2813RgbStrip(PIN, COUNT)
​
#boolean variable for determing running or not
turnedOn = True
​
​
# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
#Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/300.0)
​
    
def callback(data):
​
    if data==1:
        turnedOn = True
        while turnedOn:
            print ('Color wipe animations.')
            colorWipe(strip, Color(0, 255, 0))  # Blue wipe
            
    if data==0:
        turnedOn = False
​
def listener():
    rospy.init_node('led_controller', anonymous=True)
    rospy.Subscriber('/spray_controller/command', Float32, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
​
if __name__ == '__main__':
    listener()
