#!/usr/bin/env python
import rospy
import sys
import os
import rospkg
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import pickle
import pandas as pd
from sklearn.linear_model import LogisticRegression
from PIL import Image as ImagePIL
import copy

DEFAULT_WEIGHTS = "yolov3-tiny-prn_best-merged.weights" #"yolov3_best-416-colored-v2.weights"
DEFAULT_CONFIGURATION = "yolov3-tiny-prn.cfg"
DEFAULT_OBJ_NAMES = "obj.names" # contains the name of the classes the models were trained on ; (0) handle (1) door
FOLDER_NAME_WITH_YOLO_MODEL = 'ML/Yolo'
LOG_REG_PATH = os.getcwd() + "/../../ML/Logr/log-reg-model-504.pkl"
IMAGE_WIDTH = 1080
IMAGE_HEIGHT = 1920


class CameraSampler:
   def __init__(self, weights, cfg):
        rospy.init_node('sampling_from_camera',anonymous=True)
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # Stores the number of frames recieved and to be processed so fat
        self.frame_id = 0 

        self.starting_time = time.time()

        # Load the log. reg. model
        self.log_reg_model = pd.read_pickle(LOG_REG_PATH)

        # Load the Yolo model
        self.load_network(weights, cfg, DEFAULT_OBJ_NAMES)

        self.image_sub = rospy.Subscriber("image", ImageMSG, self.callback)
 


   def load_network(self, weights, cfg, obj):
        rospack = rospkg.RosPack()
        dr_phil_package_path = rospack.get_path('dr_phil_gazebo')
        weights_path = os.path.join(dr_phil_package_path, FOLDER_NAME_WITH_YOLO_MODEL, 'weights', weights)
        cfg_path = os.path.join(dr_phil_package_path, FOLDER_NAME_WITH_YOLO_MODEL,'cfg', cfg)
        obj_path = os.path.join(dr_phil_package_path, FOLDER_NAME_WITH_YOLO_MODEL, obj)
        self.net = cv2.dnn.readNet(weights_path,cfg_path)
        self.classes = []
        with open(obj_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layer_names = self.net.getLayerNames()
        self.outputlayers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        
        
   

   def callback(self,rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Flatten the image so it can be passed into the logr model
        pilImage = ImagePIL.fromarray(self.rgb_image)
        frame_array = np.asarray(pilImage.resize((896, 504))).reshape(1, -1)

        # Classify the frame
        log_reg_prediction = self.log_reg_model.predict(frame_array)[0]

        # If the log. reg. model thinks there is a door in the frame it calls yolo
        if log_reg_prediction:
            self.yolov3(self.rgb_image)

       
       
     
   def yolov3(self, frame):

        font = cv2.FONT_HERSHEY_PLAIN

        self.frame_id += 1
        
        # frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        height, width, channels = frame.shape
        # detecting objects
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (128, 128), (0, 0, 0), True, crop=False)  # reduce 416 to 320

        self.net.setInput(blob)
        outs = self.net.forward(self.outputlayers)
        # print(outs[1])

        # Showing info on screen/ get confidence score of algorithm in detecting an object in blob
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.3:
                    print(str(class_id) + " " + str(confidence) )
                    # onject detected
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)

                    # cv2.circle(img,(center_x,center_y),10,(0,255,0),2)
                    # rectangle coordinates
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

                    boxes.append([x, y, w, h])  # put all rectangle areas
                    # how confidence was that object detected and show that percentage
                    confidences.append(float(confidence))
                    # name of the object tha was detected
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.6)

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                #color = colors[class_ids[i]]
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 0), 2)
                cv2.putText(frame, label+" "+str(round(confidence, 2)),
                            (x, y+30), font, 1, (255, 255, 255), 2)

        elapsed_time = time.time() - self.starting_time
        fps = self.frame_id/elapsed_time
        cv2.putText(frame, "FPS:"+str(round(fps, 2)),
                    (10, 50), font, 2, (0, 0, 0), 1)

        cv2.imshow("Image", frame)
        # wait 1ms the loop will start again and we will process the next frame
        key = cv2.waitKey(1)

        #if key == 27:  # esc key stops the process
        #    break

    
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
  camera_parser = CameraSampler(weights,cfg)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
   
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)
