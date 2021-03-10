#! /usr/bin/env python3

import unittest
import sys
import numpy as np
from dr_phil_hardware.vision.ray import Ray
import math

PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class VisionUtilsTest(unittest.TestCase):

    def test_invert_homog_mat_identity_4x4(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat

        identity = np.array([[1,0,0,0],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])

        output = invert_homog_mat(identity)

        self.assertTrue(np.allclose(identity,output),"The inverse of identity:\n {},\n was:\n {}".format(identity,output))
    
    def test_invert_homog_mat_identity_3x3(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat

        identity = np.array([[1,0,0],
                             [0,1,0],
                             [0,0,1]])

        output = invert_homog_mat(identity)

        self.assertTrue(np.allclose(identity,output),"The inverse of identity:\n {},\n was:\n {}".format(identity,output))
    
    def test_invert_homog_mat_4x4_1(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat
        
        input = np.array(   [[1,0,0,0],
                             [0,0,-1,0],
                             [0,1,0,3],
                             [0,0,0,1]])

        inverse = np.array( [[1,0,0,0],
                             [0,0,1,-3],
                             [0,-1,0,0],
                             [0,0,0,1]])


        output = invert_homog_mat(input)

        self.assertTrue(np.allclose(output,inverse),"The inverse of input:\n {},\n was:\n {}".format(input,output))

    def test_invert_homog_mat_4x4_1_reverse(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat
        
        input = np.array( [[1,0,0,0],
                             [0,0,1,-3],
                             [0,-1,0,0],
                             [0,0,0,1]])

        inverse = np.array(   [[1,0,0,0],
                             [0,0,-1,0],
                             [0,1,0,3],
                             [0,0,0,1]])


        output = invert_homog_mat(input)

        self.assertTrue(np.allclose(output,inverse),"The inverse of input:\n {},\n was:\n {}".format(input,output))
    
    def test_invert_homog_mat_4x4_2(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat
        
        input = np.array(   [[0,0,-1,1],
                             [-1,0,0,2],
                             [0,-1,0,3],
                             [0,0,0,1]])

        inverse = np.array( [[0,-1,0,2],
                             [0, 0,-1,3],
                             [-1,0,0,1],
                             [0,0,0,1]])


        output = invert_homog_mat(input)

        self.assertTrue(np.allclose(output,inverse),"The inverse of input:\n {},\n was:\n {}".format(input,output))

    def test_invert_homog_mat_4x4_2_reverse(self):
        from dr_phil_hardware.vision.utils import invert_homog_mat
        
        input = np.array( [[0,-1,0,2],
                             [0, 0,-1,3],
                             [-1,0,0,1],
                             [0,0,0,1]])
        inverse = np.array(   [[0,0,-1,1],
                             [-1,0,0,2],
                             [0,-1,0,3],
                             [0,0,0,1]])



        output = invert_homog_mat(input)

        self.assertTrue(np.allclose(output,inverse),"The inverse of input:\n {},\n was:\n {}".format(input,output))

    def test_subtract(self):
        from dr_phil_hardware.vision.utils import subtract

        ray1 = Ray(np.array([0, 0, 0]), np.array([1, 0, 0]), length=1)
        ray2 = Ray(np.array([0, 0, 0]), np.array([0, 0, 1]), length=1)

        subtr = subtract(ray1, ray2)
        self.assertTrue(np.array_equal(subtr.origin, ray2.get_point()))
        self.assertTrue(subtr.length == math.sqrt(2))

    
if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG,'vision_utils_test',VisionUtilsTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest