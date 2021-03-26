#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.ray import Ray
from shapely.geometry import LineString
import math 
from tf import transformations as t

from typing import Tuple



def cartesian_line_from_ray(ray : Ray):
    """ returns tuple (m,c) with m being gradient and c y-interecept"""
    mx = ray.dir[0,0]
    my = ray.dir[1,0]
    bx = ray.origin[0,0]
    by = ray.origin[1,0]
    m = my/mx 
    c = by - (m * bx)
    return (m,c)
    
def cartesian_2d_line_intercept(line1 :Tuple[float,float],line2:Tuple[float,float]):
    """[summary]
    https://math.stackexchange.com/questions/25171/intersection-of-two-lines-in-2d
    Args:
        line1 (Tuple[float,float]): [description]
        line2 (Tuple[float,float]): [description]
    Returns:
        2d array [[x],[y]]
    """
    m1 = line1[0]
    c1 = -line1[1]
    b1 = -1
    m2 = line2[0]
    c2 = -line2[1]
    b2 = -1
    bottom = (m1*b2 - b1*m2)

    if bottom == 0: # no unique solution 
        return None
    else:
        x = (c1*b2  - b1*c2)/bottom
        y = (m1*c2 - c1*m2)/bottom
        return np.array([[x],[y]])

def quat_from_yaw(yaw):
    """ finds and returns the quaternion corresponding to rotation of vector (1,0,0) by the given yaw around the z axis"""
    return t.quaternion_from_matrix(t.rotation_matrix(yaw,np.array([0,0,1])))

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
    """ Returns the angle in radians between vectors 'v1' and 'v2' in 0 to pi range::
            >>> angle_between([[1], [0], [0]], [[0], [1], [0]])
            1.5707963267948966
            >>> angle_between([[1], [0], [0]], [[1], [0], [0]])
            0.0

    """
    angle = float(np.arccos((v1.T @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))
    return angle if angle < math.pi else (math.pi * 2) - angle

def angle_between_pi(v1, v2,plane_normal):
    """ Returns the angle in radians between vectors 'v1' and 'v2' but in full 0 to 2pi range::

            >>> angle_between([[1], [0], [0]], [[0], [1], [0]])
            1.5707963267948966
            >>> angle_between([[1], [0], [0]], [[1], [0], [0]])
            0.0

    """

    dot = v1.T @ v2 
    angle = float(np.arccos((v1.T @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))))

    cross = np.cross(v1[:,0],v2[:,0])

    if np.dot(cross,plane_normal) < 0:
        angle *= -1 

    return angle % (math.pi * 2)
        