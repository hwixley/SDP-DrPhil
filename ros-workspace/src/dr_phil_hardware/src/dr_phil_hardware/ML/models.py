#!/usr/bin/env python
import sys
import os
import rospkg
import cv2
import time
import numpy as np
#import pickle
#import pandas as pd
#from sklearn.linear_model import LogisticRegression
#from PIL import Image as ImagePIL
#import copy

DEFAULT_WEIGHTS =  "yolov3-tiny-prn_best-merged.weights" 
DEFAULT_CONFIGURATION = "yolov3-tiny-prn.cfg"
DEFAULT_OBJ_NAMES = "obj.names" # contains the name of the classes the models were trained on ; (0) handle (1) door
FOLDER_NAME_WITH_YOLO_MODEL = 'ML/Yolo'
ROS_PACKAGE_NAME = 'dr_phil_hardware'
#LOG_REG_PATH = os.getcwd() + "/../../ML/Logr/log-reg-model-504.pkl"
#LOG_REG_IMAGE_WIDTH = 1080
#LOG_REG_IMAGE_HEIGHT = 1920
IMAGE_YOLO_INPUT = (320,320)

#These variables are used to determine the time it takes to get yolo results back  
frame_id = 0 #Number of frames processed so far
starting_time = time.time() #Store initial time


# x is x-position in image; y is y-position in image y; w is width of the bounding box; h is height of the bounding box ; 
# confidence_score tells us how confident the object is in fact that label
class YOLOBoundingBox:
    def __init__(self,x, y, w, h, label, confidence):
        self.label = label
        self.x=x
        self.y=y
        self.width=w
        self.height=h
        self.confidence_score = confidence


# Note: It is much slower to load the network and the classes everytime yolov3 is called. The function can default to loading them for you.
# However, it is recommended to call the function load_network_and_classes yourself, in order to store the network model and retrieve the class names from a file of .names format earlier in your program before calling this function to increase efficiency.   
def yolov3(image, network=None, output_layers=None, class_names = None, weights=DEFAULT_WEIGHTS,cfg=DEFAULT_CONFIGURATION, obj=DEFAULT_OBJ_NAMES):
    #If any of network, output_layers, and class names are not provided in the argument
    if network is None or output_layers is None or class_names is None:
        print("Loading the YOLO Model")
        network, output_layers, class_names = load_network_and_classes(weights,cfg, obj)

    results = make_predictions(image, network, output_layers, class_names)
    return results

#Load the YOLO model and get the appropriate classes that it was trained on
def load_network_and_classes(weights=DEFAULT_WEIGHTS, cfg=DEFAULT_CONFIGURATION, obj=DEFAULT_OBJ_NAMES):
    rospack = rospkg.RosPack()
    dr_phil_package_path = rospack.get_path(ROS_PACKAGE_NAME)
    
    net, output_layers = load_network(weights,cfg,dr_phil_package_path)
    classes = load_classes(obj,dr_phil_package_path)

    return net, output_layers, classes


def load_network(weights, cfg, package_path):
    weights_path = os.path.join(package_path, FOLDER_NAME_WITH_YOLO_MODEL, 'weights', weights)
    cfg_path = os.path.join(package_path, FOLDER_NAME_WITH_YOLO_MODEL,'cfg', cfg)
    # load the YOLO network model into OpenCV
    net = cv2.dnn.readNet(weights_path,cfg_path)
    #Get the name of all layers of the YOLO neural network.
    layer_names = net.getLayerNames()
    #Get the index of the output layers.
    indexOutputLayers = net.getUnconnectedOutLayers()
    #Get the corresponding indicies 
    output_layers = [layer_names[i[0] - 1] for i in indexOutputLayers]

    return net, output_layers

#Load the classes that the model were trained on
def load_classes(obj, package_path):
    obj_path = os.path.join(package_path, FOLDER_NAME_WITH_YOLO_MODEL, obj)
    classes=[]
    with open(obj_path, "r") as f:
        classes = [line.strip() for line in f.readlines()]
    return classes


def make_predictions(frame, net, output_layers, classes):
    height, width, channels = frame.shape
    #Pre-processing: 
    # The input to the network is called a blob object. 
    # Transform image into blob (i.e preprocess the image sutiable for Neural Network model) with specified image input size IMAGE_YOLO_INPUT to pass it through the model.
    #This pre-processing includes Mean subtraction and Scaling by some factor
    blob = cv2.dnn.blobFromImage(
        frame, 0.00392, IMAGE_YOLO_INPUT, (0, 0, 0), True, crop=False)  # reduce 416 to 320
    #Pass the blob object, which is the image pre-processed, to the network:
    net.setInput(blob)
    #Vectors which contain the results of the model
    outs = net.forward(output_layers)

    #Now, extract the output information returned by the model
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.3:                
                #object detected- calculate the center of the enclosed object (the center enclosed by the predicted bounding box)
                center_x = int(detection[0]*width)
                center_y = int(detection[1]*height)
                #cv2.circle(frame,(center_x,center_y),10,(0,255,0),2)

                #retrieve the rectangle bounding box enclosing corresponding object
                w = int(detection[2]*width)
                h = int(detection[3]*height)
                x = int(center_x - w/2)
                y = int(center_y - h/2)
                #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

                boxes.append([x, y, w, h])  # put all rectangle areas
                # how confident was that object detected and show that percentage
                confidences.append(float(confidence))
                # name of the object that was detected
                class_ids.append(class_id)

    return post_process(boxes,confidences,classes, class_ids)




#Post-Processing: Performs non-maximum suppresion to eliminate overlapping results 
def post_process(boxes,confidences, class_names, class_ids):
    indexes =  cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.6)
    results = []
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(class_names[class_ids[i]])
            confidence = confidences[i]

            prediction = YOLOBoundingBox(x,y,w,h,label,confidence)

            results.append(prediction)
       

    return results


#Show the bounding boxes returned by the YOLO model post-processing. 
#You can also see how fast the process is if the function is provided with initial time of the program and the frame_id, to help calculate the number of frames per second (FPS).
def visualise_results(frame, results, starting_time=None, frame_id=None):
    font = cv2.FONT_HERSHEY_PLAIN

    for prediction in results:
        cv2.rectangle(frame, (prediction.x, prediction.y), (prediction.x+prediction.width, prediction.y + prediction.height), (255, 255, 0), 2)
        cv2.putText(frame, prediction.label+" "+str(round(prediction.confidence_score, 2)),
                    (prediction.x, prediction.y+30), font, 1, (255, 255, 255), 2)


    if starting_time is not None and frame_id is not None:
        elapsed_time = time.time() - starting_time
        fps = frame_id/elapsed_time
        cv2.putText(frame, "FPS:"+str(round(fps, 2)),
                    (10, 50), font, 2, (0, 0, 0), 1)

    # cv2.imshow("Object Detection Results", frame)
    # wait 1ms the loop will start again and we will process the next frame
    key = cv2.waitKey(1)


#DEPRECATED: Was used as classifer before YOLO is called to determine if an image has a door or not before proceeding to determine where it is in the image.
#The process turned out to give worse results, so YOLOv3 was used for the robot model
def log_reg(self):
    pass
    # Flatten the image so it can be passed into the logr model
    #pilImage = ImagePIL.fromarray(self.rgb_image)
    #frame_array = np.asarray(pilImage.resize((896, 504))).reshape(1, -1)

    # Classify the frame
    #log_reg_prediction = self.log_reg_model.predict(frame_array)[0]
    #print(self.log_reg_model.predict_proba(frame_array))
    #print(log_reg_prediction)

    # If the log. reg. model thinks there is a door in the frame it calls yolo
    #if log_reg_prediction:
    #    self.yolov3(self.rgb_image)
    #    print("yes")
    
    #else:



        








        

    

