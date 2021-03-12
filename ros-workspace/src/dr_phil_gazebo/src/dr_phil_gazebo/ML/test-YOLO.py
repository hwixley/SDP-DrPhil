#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import sys
import time
import cv2
import numpy as np

from models import yolov3, DEFAULT_WEIGHTS, DEFAULT_CONFIGURATION, DEFAULT_OBJ_NAMES, visualise_results, load_network_and_classes

class TestYOLO:
   def __init__(self, weights, cfg):
        rospy.init_node('test_YOLO',anonymous=True)
        self.weights = weights
        self.cfg = cfg
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

   
        # Stores the number of frames recieved and to be processed so far
        self.frame_id = 0
        #Store starting time to keep track of elapsed time
        self.starting_time = time.time()
        #Load the network and classes earlier - Done in order to improve efficiency of the node to process images quicker instead of taking time to load
        self.net, self.out, self.classes = load_network_and_classes(self.weights, self.cfg)
     
        #Initialise image subscriber and call callback function
        self.image_sub = rospy.Subscriber("image", ImageMSG, self.callback)
              
   

   def callback(self,rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            #Increment the number of frames to be processed
            self.frame_id += 1
        except CvBridgeError as e:
            print(e)

        #Pass the image and the appropriate arguments to get results
        results = yolov3(self.rgb_image,weights=self.weights,cfg=self.cfg, network=self.net, output_layers=self.out, class_names=self.classes)
        
        
        #See the YOLO results by calling the visualise results function
        visualise_results(self.rgb_image, results, self.starting_time, self.frame_id)

      

    
# call the class
def main(args):
  print("Running Node...")
  print()
  weights = ""
  cfg = ""
  try:
      weights = args[1]
      cfg = args[2]
  except:
    weights = DEFAULT_WEIGHTS
    cfg = DEFAULT_CONFIGURATION
    print("If you want to use other weights and cfg (make sure they're the suitable cfg file for the weights), please add them to the Yolo Folder inside ML and run the node with the following format:")
    print("rosrun dr-phil <filename-in-ML>.weights <filename-in-ML-folder>.cfg")
    print("Default model was trained on two classes: (0) handle ; (1) door - See obj.names")
    print()
    print("Using default weights and configuraton...")


      
  print("weights: " + weights)
  print("cfg: " + cfg)
  print("Classes names contained in: " + DEFAULT_OBJ_NAMES)
  camera_parser = TestYOLO(weights,cfg)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
   
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)
