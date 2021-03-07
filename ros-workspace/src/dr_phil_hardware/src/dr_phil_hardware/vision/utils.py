#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.ray import Ray
from shapely.geometry import LineString
import math 
from tf import transformations as t

def invert_homog_mat(hm):
    """ inverts homogenous matrix expressing rotation and translation in 3D or 2D """  
    
    return t.inverse_matrix(hm)

    # # if 3D transformation matrix
    # if hm.shape[0] == 4 == hm.shape[1]:
    #     newH = 3
    # elif hm.shape[0] == 3 == hm.shape[1]:
    #     newH = 2
    # else:
    #     raise Exception("Invalid matrix shape: " + hm.shape)


    # nHm = np.zeros((newH+1,newH+1))

    # rotI = hm[0:newH,0:newH].T
    # transI = rotI @ -hm[0:newH,newH]

    # nHm[0:newH,0:newH] = rotI
    # nHm[0:newH,newH] = transI
    # nHm[newH,newH] = 1
    # return nHm

def intersect(ray1 : Ray, ray2 : Ray):
    """ returns true if 2D rays intersect, false otherwise """

    segment1 = LineString([tuple(ray1.origin),tuple(ray1.dir * ray1.length)])
    segment2 = LineString([tuple(ray2.origin),tuple(ray2.dir * ray2.length)])

    return segment1.intersects(segment2)

def subtract(ray1, ray2):
    """ returns ray1 - ray2
    the 2 rays have the same origin """
    
    origin = ray2.origin + ray2.dir
    dir = np.subtract(ray1.origin + ray1.dir, ray2.origin + ray2.dir)
    length = math.sqrt(ray1.length ** 2 + ray2.length ** 2)

    return Ray(origin, dir, length)
    