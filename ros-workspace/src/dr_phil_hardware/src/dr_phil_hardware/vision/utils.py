#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.ray import Ray
from shapely.geometry import LineString
import math 
from tf import transformations as t

def invert_homog_mat(hm):
    """ inverts homogenous matrix expressing rotation and translation in 3D or 2D """  
    
    return t.inverse_matrix(hm)



def intersect(ray1 : Ray, ray2 : Ray):
    """ returns true if 2D rays intersect, false otherwise """

    segment1 = LineString([list(ray1.origin),list(ray1.get_point())])
    segment2 = LineString([list(ray2.origin),list(ray2.get_point())])
    return segment1.intersects(segment2)

def subtract(ray1, ray2):
    """ returns ray1 - ray2
    the 2 rays have the same origin, and have finite length"""
    
    origin = ray2.get_point()
    dir = ray1.get_point() - origin
    length = np.linalg.norm(dir)

    return Ray(origin, dir, length)
    
def interpolated_ray(ray1: Ray,ray2: Ray,r, newL):
    """ given rays with the same origin, returns the ray r of the way between the tip of ray1 and the tip of ray2 with given length
        Args:
            ray1: the start ray
            ray2: the end ray
            r: the ratio of the distance to interpolate between the tips
            newL: the length to give to the new ray
    """

    # need to have same origin
    assert(np.allclose(ray1.origin,ray2.origin))

    tip1 = ray1.get_point()
    tip2 = ray2.get_point()

    # get interpolation direction
    dir = tip2 - tip1
    
    new_tip = tip1 + (dir * r)
    new_dir = new_tip - ray1.origin

    return Ray(ray1.origin,new_dir,newL)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
            >>> angle_between([[1], [0], [0]], [[0], [1], [0]])
            1.5707963267948966
            >>> angle_between([[1], [0], [0]], [[1], [0], [0]])
            0.0

    """
    angle = float(np.arccos((v1.T @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))
    return angle if angle < math.pi else (math.pi * 2) - angle

def angle_between_pi(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between([[1], [0], [0]], [[0], [1], [0]])
            1.5707963267948966
            >>> angle_between([[1], [0], [0]], [[1], [0], [0]])
            0.0

    """
    angle = float(np.arccos((v1.T @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))
    return angle