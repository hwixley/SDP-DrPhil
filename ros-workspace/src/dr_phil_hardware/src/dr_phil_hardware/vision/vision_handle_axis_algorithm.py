#!/usr/bin/python3

import cv2
import numpy as np
import sys

PINK = [233,42,222]
BLACK = [0,0,0]


def elongation(m):
    x = m['mu20'] + m['mu02']
    y = 4 * m['mu11']**2 + (m['mu20'] - m['mu02'])**2
    if (x - y**0.5) != 0:
        return (x + y**0.5) / (x - y**0.5)
    return 0

class Handle():
    PULL_DOOR = 1 #default for define handle_features_heuristic
    LEVER_HANDLE = 2 


""" Given an image of a handle encapsulated in a bounding box, we will find the
locations of surface handle where the axis of rotation is located at (if any rotation - depends on handle type)
and the part of the handle in which the robot will apply force on (to be able to open
the door).
This is a simple heuristic with the following assumptions:
 * The robot is well-localised, placing itself in a certain position from the door
handle to apply the heuristic function successfully.
 * The bounding box resulting from using an ML Object Classifier is accurate
 and places the bounding box at the exact or edges of the handle detected.
 * The camera is rigidly mounted, such that the door handle is always in a horizontal
 line at its stationary point.
 
 Default Type of Handle: Handle.PULL_DOOR
 
"""
 #TODO: The function currently uses preconfigured constants for each type of door handle. However, you can choose to have simplistic= False in order to use the outline of the handle to determine its location (Implemented for Pull doors only)
 # --- Still in BETA STAGE: Make sure contours depict the shape of the handle and nothing else. Seems to work somewhat well in simulation. However, fails to work in images with high texture or complex handle structure.
 
def define_handle_features_heursitic(image, top_left, width, height, handle_type = Handle.PULL_DOOR, simplistic=True):            
    sigma= 0.33 #Used 
    if (handle_type == Handle.LEVER_HANDLE): #2 outputs - p1, p2
        gamma=0.8 
        beta=0.1
        alpha=0.05
        handle_rotation_axis = int(top_left[0] + alpha*width), int(top_left[1] + beta*height)
        handle_force_location = int(top_left[0] + gamma*width), int(top_left[1] + beta*height)
        return handle_rotation_axis, handle_force_location
    else: #Handle.PULL_DOOR; 1 output - p2 only
        handle_rotation_axis = None
        handle_force_location = None
        gamma=0.5
        beta=0.5
        threshold_difference = 30 #Used to make sure that the center returned by calculating the edges isn't too far.
        if (simplistic):
            handle_force_location = (top_left[0] + gamma*width, top_left[1] + beta*height)
            handle_rotation_axis = None
        else:
            #Since the shape of pull doors are less complex and we are only interested in the center point of the handle, we could 
            #determine the outline of the handle and find the centroid of that outline.
            gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 3)
            v = np.median(blurred)
                        
            lower = int(max(0, (1.0 - sigma) * v))
            upper = int(min(255, (1.0 + sigma) * v))
            canny_edge = cv2.Canny(gray, lower, upper)
            cv2.imshow('canny', canny_edge)
            contours, hierarchy = cv2.findContours(canny_edge.copy(),cv2.RETR_EXTERNAL,1)
            
            #TODO: Make sure that the contour returned correctly depicts that of the handle, for all contours found in an image. It doesnt work well for other pictures with a lot of texture.
            center_X = 0
            center_Y = 0
            x,y,w,h = 0,0,0,0
            max_elong = 0
            for cnt in contours:
                M = cv2.moments(cnt)
                elong = elongation(M) #We will assess based on elongation. In a cropped image, the shape with the highest elongation probably belongs to the outline of the handle
                if elong > max_elong and M["m00"]!=0:
                    max_elong = elong
                    x,y,w,h = cv2.boundingRect(cnt)

                    center_X = int(x+ (w/2))
                    center_Y = int(y+ (h/2))
                

            #cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.putText(image, "center", (center_X - 20, center_Y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                  
            
            #As a final check, if the centers found is way off, default to getting the center of the image.
            if (abs(center_X - (top_left[0] + gamma*width)) > threshold_difference or abs(center_Y - (top_left[1] + beta*height)) > threshold_difference):     
                handle_force_location = (int(top_left[0] + gamma*width),int(top_left[1] + beta*height))
            else:
                handle_force_location = (center_X, center_Y)
            
 
            
        return handle_rotation_axis , handle_force_location

"""
Pinpoint the locations resulted from defining handle_features_heuristic.
For convention, p1 is the handle rotation axis and p2 is the location where
you apply force to the handle.
"""
def visualise_features(img, point1, point2):
    print("Coordinates of the points")
    print(point1)
    print("p2")
    print(point2)
    #Show the locations of the points by pinpointing it with an intersection of
    #two different lines.
    cv2.circle(img, (point1[0], point1[1]), 7, PINK, -1)
    cv2.circle(img, (point2[0], point2[1]), 7, BLACK, -1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    #Create text next to the points pinpointed to further identify what the intersection indicates
    cv2.putText(img, 'p1', (int(point1[0]-2),int(point1[1]-5)), font, 0.7, (0, 255, 0),1, cv2.LINE_AA)
    cv2.putText(img, 'p2', (int(point2[0]-2),int(point2[1]-5)), font, 0.7, (0, 255, 0),1, cv2.LINE_AA)
    cv2.imshow("Image of Handle", img)
    print("To close the image, press any key on your keyboard (On the opened image)!")
    cv2.waitKey(0)
"""
Pinpoint the locations resulted from defining handle_features_heuristic.
For convention, p2 is the location where you apply force to the handle. 
Used when we only have one point - i.e no axis of rotation like pull door handles that have no lever.
"""   
def visualise_features_p2_only(img, point2):
    print("Coordinates of p2: ")
    print(point2)
    #Show the locations of the points by pinpointing it with an intersection of two different lines.
    cv2.circle(img, (point2[0], point2[1]), 7, PINK, -1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    #Create text next to the points pinpointed to further identify what the intersection indicates
    cv2.putText(img, 'p2', (int(point2[0]-2),int(point2[1]-5)), font, 0.7, (0, 255, 0),1, cv2.LINE_AA)
    cv2.imshow("Image of Handle", img)
    print("To close the image, press any key on your keyboard (On the opened image)!")
    cv2.waitKey(0)

#Testing
if __name__=="__main__":
   #Test images to be used
    handle_img= cv2.imread("images/pull-door-1.png",1)
    top_left = (0,0)
    height=handle_img.shape[0]
    width = handle_img.shape[1]
    gamma = 0.8
    beta = 0.1
    alpha = 0.05
    handle_type = Handle.PULL_DOOR
    

    
    
    #Check that the number of outputs returned is correct as 
    rotation_point, force_location = define_handle_features_heursitic(handle_img, top_left,width,height,handle_type,simplistic=False)
    if (rotation_point is None):
        visualise_features_p2_only(handle_img,force_location)
    else:
        visualise_features(handle_img,rotation_point,force_location)
