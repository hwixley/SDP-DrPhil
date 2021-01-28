#!/usr/bin/python3

import cv2
import numpy as np
PINK = [233,42,222]
BLACK = [0,0,0]

""" Given an image of a handle encapsulated in a bounding box, we will find the
locations of surface handle where the axis of rotation is located at and
the part of the handle in which the robot will apply force on (to be able to open
the door).
This is a simple heuristic with the following assumptions:
 * The robot is well-localised, placing itself in a certain position from the door
handle to apply the heuristic function successfully.
 * The bounding box resulting from using an ML Object Classifier is accurate
 and places the bounding box at the exact or edges of the handle detected.
 * The camera is rigidly mounted, such that the door handle is always in a horizontal
 line at its stationary point.
"""
def define_handle_features_heursitic(top_left, width, height, gamma=0.05,beta=0.1,alpha=0.1):
    handle_rotation_axis = (top_left[0] + alpha*width, top_left[1] + beta*height)
    handle_force_location = (top_left[0] + gamma*width, top_left[1] + beta*height)
    return handle_rotation_axis, handle_force_location

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
    img[int(point1[1]),:] = PINK
    img[:,int(point1[0])] = PINK
    img[int(point2[1]),:] = BLACK
    img[:,int(point2[0])] = BLACK
    font = cv2.FONT_HERSHEY_SIMPLEX
    #Create text next to the points pinpointed to further identify what the intersection indicates
    cv2.putText(img, 'p1', (int(point1[0]-2),int(point1[1]-5)), font, 0.7, (0, 255, 0),1, cv2.LINE_AA)
    cv2.putText(img, 'p2', (int(point2[0]-2),int(point2[1]-5)), font, 0.7, (0, 255, 0),1, cv2.LINE_AA)
    cv2.imshow("Image of Handle", img)
    cv2.waitKey(0)

#Testing
if __name__=="__main__":
    handle_img= cv2.imread("handle-2.png",1)
    top_left = (0,0)
    height=handle_img.shape[0]
    width = handle_img.shape[1]
    gamma = 0.8
    beta = 0.1
    alpha = 0.05
    rotation_point, force_location = define_handle_features_heursitic(top_left,width,height,  gamma, beta, alpha)
    visualise_features(handle_img,rotation_point,force_location)
