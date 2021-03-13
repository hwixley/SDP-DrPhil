#! /usr/bin/env python3

import unittest
import sys
import numpy as np
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.vision.utils import subtract,intersect,invert_homog_mat,interpolated_ray,angle_between
from dr_phil_hardware.test.test_utils.test_utils import assertRayEquals

import math

PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class VisionUtilsTest(unittest.TestCase):

    def test_invert_homog_mat_identity_4x4(self):
        identity = np.array([[1,0,0,0],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])

        output = invert_homog_mat(identity)

        self.assertTrue(np.allclose(identity,output),"The inverse of identity:\n {},\n was:\n {}".format(identity,output))
    
    def test_invert_homog_mat_identity_3x3(self):
        identity = np.array([[1,0,0],
                             [0,1,0],
                             [0,0,1]])

        output = invert_homog_mat(identity)

        self.assertTrue(np.allclose(identity,output),"The inverse of identity:\n {},\n was:\n {}".format(identity,output))
    
    def test_invert_homog_mat_4x4_1(self):        
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

    def test_subtract_length_square(self):

        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[1], [0], [0]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [1]]), length=1)

        subtr = subtract(ray1, ray2)
        self.assertTrue(subtr.length == math.sqrt(2))

    def test_subtract_vector_square(self):
        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[1], [0], [0]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [1], [0]]), length=1)

        ray_correct = Ray(ray2.get_point(),np.array([[1],[-1],[0]]),length=math.sqrt(2))
        assertRayEquals(self,ray_correct,subtract(ray1,ray2),msg="subtracted ray was off")

    def test_intersect(self):
        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[1], [0], [0]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [1], [0]]), length=1)

        self.assertTrue(intersect(ray1,ray2),"these rays should intersect")

    def test_no_intersect(self):
        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[1], [0], [0]]), length=1)
        ray2 = Ray(np.array([[0], [1], [0]]), np.array([[0], [1], [0]]), length=1)

        self.assertFalse(intersect(ray1,ray2),"these rays should not intersect")

    def test_no_intersect_2(self):
        ray1 = Ray(np.array([[1], [0], [0]]), np.array([[-1], [0], [0]]), length=0.9)
        ray2 = Ray(np.array([[0], [1], [0]]), np.array([[0], [-1], [0]]), length=0.9)

        self.assertFalse(intersect(ray1,ray2),"these rays should not intersect")

    def test_interpolated_ray_1(self):

        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [1]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [-1]]), length=1)

        ray_correct = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [0]]), length=0)

        assertRayEquals(self,ray_correct,interpolated_ray(ray1,ray2,0.5,0))

    def test_interpolated_ray_2(self):

        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [1]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [-1]]), length=1)

        ray_correct = ray1

        assertRayEquals(self,ray_correct,interpolated_ray(ray1,ray2,0,1))

    def test_interpolated_ray_3(self):

        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [1]]), length=1)
        ray2 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [-1]]), length=1)

        ray_correct = ray2

        assertRayEquals(self,ray_correct,interpolated_ray(ray1,ray2,1,1))

    def test_interpolated_ray_diff_origin(self):

        ray1 = Ray(np.array([[0], [0], [0]]), np.array([[0], [0], [1]]), length=1)
        ray2 = Ray(np.array([[0.00000000001], [0], [0]]), np.array([[0], [0], [-1]]), length=1)

        try:
            interpolated_ray(ray1,ray2,0.5,1)
            self.assertTrue(False,"Rays with different origins should not be allowed, exception wasn't thrown")
        except:
            self.assertTrue(True)
        
    def test_angle_between_1_2_quadrant(self):
        self.assertAlmostEqual(math.pi/2,angle_between(np.array([[1],[0],[0]]),np.array([[0],[1],[0]]) ))

    def test_angle_between_1_quadrant(self):
        self.assertAlmostEqual(0,angle_between(np.array([[1],[0],[0]]),np.array([[1],[0],[0]]) ))

    def test_angle_between_2_quadrant(self):
        self.assertAlmostEqual((math.pi*3)/4,angle_between(np.array([[1],[0],[0]]),np.array([[-0.5],[0.5],[0]]) ))
        
    def test_angle_between_3_quadrant(self):
        self.assertAlmostEqual((math.pi*3)/4,angle_between(np.array([[1],[0],[0]]),np.array([[-0.5],[-0.5],[0]]) ))

    def test_angle_between_4_quadrant(self):
        self.assertAlmostEqual(math.pi/4,angle_between(np.array([[1],[0],[0]]),np.array([[0.5],[-0.5],[0]]) ))

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG,'vision_utils_test',VisionUtilsTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest