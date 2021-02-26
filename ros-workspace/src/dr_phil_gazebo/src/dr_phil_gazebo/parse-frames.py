#!/usr/bin/env python
from __future__ import absolute_import
import rospy
import sys
import os
import rospkg
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image as ImagePIL
# listens to the "joint_states" topic and sends them to "joint_callback" for processing


class CameraSampler(object):
   def __init__(self):
        rospy.init_node(u'sampling_from_camera',anonymous=True)
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.count = 0 
        self.index = 0
        rospack = rospkg.RosPack()
        self.dr_phil_package_path = rospack.get_path(u'dr-phil')
        self.image_sub = rospy.Subscriber(u"image",ImageMSG, self.callback)

   def callback(self,rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding=u"bgr8")
        except CvBridgeError, e:
            print e

        
        if self.count % 20 == 0:
           im_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
           img = ImagePIL.fromarray(im_rgb)
           new_image_path = os.path.join(self.dr_phil_package_path,u'frames', u"%d.jpg" % self.index) 
           img = img.save(new_image_path)
           print u"Image %d.jpg saved in frames" % self.index 
           self.index += 1

        self.count += 1

     
    
# call the class
def main(args):
  print u"Running Node..."
  camera_parser = CameraSampler()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print u"Shutting down"
   
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == u'__main__':
    main(sys.argv)