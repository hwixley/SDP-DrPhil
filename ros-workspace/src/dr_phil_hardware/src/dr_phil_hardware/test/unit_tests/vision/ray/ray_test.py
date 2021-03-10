#! /usr/bin/env python3

import unittest
import sys
import rospy 
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from dr_phil_hardware.vision.lidar import Lidar
import tf
from tf2_msgs.msg import TFMessage
from dr_phil_hardware.test.test_utils.test_utils import assertRayEquals 
from dr_phil_hardware.vision.ray import Ray
import numpy as np
import math


PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class RayTest(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("ray_test",anonymous=True)

    def test_get_transformed_trans_orig_zero(self):
        ray = Ray(np.array([[0],[0],[0]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[1],[0],[0]]),np.array([[1],[0],[0]]),length=2)
        transformation = np.array([[1,0,0,1],
                                   [0,1,0,0],
                                   [0,0,1,0],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))
    
    def test_get_transformed_rot_orig_zero(self):
        ray = Ray(np.array([[0],[0],[0]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[0],[0],[0]]),np.array([[0],[0],[1]]),length=2)
        transformation = np.array([[0,0,0,0],
                                   [0,1,0,0],
                                   [1,0,1,0],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))

    def test_get_transformed_rot_1(self):
        ray = Ray(np.array([[1],[2],[3]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[3],[2],[1]]),np.array([[0],[0],[1]]),length=2)
        transformation = np.array([[0,0,1,0],
                                   [0,1,0,0],
                                   [1,0,0,0],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))

    def test_get_transformed_rot_2(self):
        ray = Ray(np.array([[1],[2],[3]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[3],[1],[2]]),np.array([[0],[1],[0]]),length=2)
        transformation = np.array([[0,0,1,0],
                                   [1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))


    def test_get_transformed_rot_trans_orig_zero(self):
        ray = Ray(np.array([[0],[0],[0]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[0],[0],[0]]),np.array([[0],[0],[1]]),length=2)
        transformation = np.array([[0,0,0,1],
                                   [0,1,0,0],
                                   [1,0,1,0],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))

    def test_get_transformed_rot_trans_1(self):
        ray = Ray(np.array([[1],[2],[3]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[4],[2],[1]]),np.array([[0],[0],[1]]),length=2)
        transformation = np.array([[0,0,1,1],
                                   [0,1,0,0],
                                   [1,0,0,0],
                                   [0,0,0,1]])


    def test_get_transformed_rot_trans_2(self):
        ray = Ray(np.array([[1],[2],[3]]),np.array([[1],[0],[0]]),length=2)
    
        correct_ray = Ray(np.array([[4],[3],[2]]),np.array([[0],[0],[1]]),length=2)
        transformation = np.array([[0,0,1,1],
                                   [0,1,0,1],
                                   [1,0,0,1],
                                   [0,0,0,1]])

        assertRayEquals(self,correct_ray,ray.get_transformed(transformation))
    
if __name__ == "__main__":
    import rostest
    rostest.unitrun(PKG,'ray_test',RayTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest