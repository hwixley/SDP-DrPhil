#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.ray import Ray
from shapely.geometry import LineString

def invert_homog_mat(hm):
    """ inverts homogenous matrix expressing rotation and translation in 3D or 2D """  
    
    # if 3D transformation matrix
    if hm.shape[0] == 4 == hm.shape[1]:
        newH = 3
    elif hm.shape[0] == 3 == hm.shape[1]:
        newH = 2
    else:
        raise Exception("Invalid matrix shape: " + hm.shape)
        
    rotI = hm[0:newH,0:newH].T
    transI = -rotI @ hm[0:newH,newH]

    hm[0:newH,0:newH] = rotI
    hm[0:newH,newH] = transI

    return hm

def intersect(segment, camera_ray):
    """ returns true if 2D rays intersect, false otherwise """
    base_start = tuple(segment.origin[:2])
    base_end = tuple(segment.dir[:2]) 
    segment = LineString([base_start, base_end])

    camera_ray_start = tuple(camera_ray.origin[:2])
    tmp_end = camera_ray.dir[:2]
    """ make sure camera_ray segment is long enough to intersect the base segment """
    while tmp_end[0] <= base_start[0]:
        tmp_end *= 2
    camera_ray_end = tuple(tmp_end)
    camera_ray = LineString([camera_ray_start, camera_ray_end])

    return camera_ray.intersects(segment)

def subtract(ray1, ray2):
    """ returns ray1 - ray2
    the 2 rays have the same origin """
    
    origin = ray2.dir
    dir = np.subtract(ray1.dir, ray2.dir)
    length = math.sqrt(ray1.length ** 2 + ray2.length ** 2)

    return Ray(origin, dir, length)
    