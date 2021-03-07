#! /usr/bin/env python3

import unittest
import sys
import numpy as np

from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.test.test_utils.test_utils import assertRayEquals 

from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
import rospy 
import tf
import math

PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class CameraTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node("camera_test",anonymous=True)


        self.robot_frame = "/base_link"
        
        # setup the camera and transforms

        try:
            self.camera_info = rospy.wait_for_message("/camera_info",CameraInfo,timeout=rospy.Duration(5))
        except:
            self.fail("Could not setup camera")

        self.camera = Camera(self.camera_info)    

        camera_transform_listener = tf.TransformListener()
        transformerRos = tf.TransformerROS()

        # this will block
        try:
            rospy.wait_for_message("/tf_static",TFMessage,timeout=rospy.Duration(5))
        except:
            self.fail("Could not setup transforms")


        # wait for transform to become available
        camera_transform_listener.waitForTransform(
            self.camera.get_frame_id(),
            self.robot_frame,
            rospy.Time.now(),
            timeout=rospy.Duration(2))

        # get transform
        (trans,rot) = camera_transform_listener.lookupTransform(
            self.camera.get_frame_id(),
            self.robot_frame,
            rospy.Time.now())
        
        try:
            self.rob2cam = transformerRos.fromTranslationRotation(trans,rot)
            self.camera.setup_transform(self.rob2cam)
        except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
            self.fail("Could not setup transforms")

    def test_center_pixel_to_camera_ray(self):

        # vector pointing forward in camera
        correct_ray = Ray(np.array([[0],[0],[0]]),np.array([[0],[0],[1]]),length=1)
        
        #notice inversion of pixels
        mid_point = self.camera.get_pixel_through_ray(correct_ray) 
        ray_img = self.camera.get_ray_through_image(mid_point)


        assertRayEquals(self,correct_ray,ray_img,
            msg="correct: \n {} \n was: \n {}".format(correct_ray,ray_img))

    def test_center_pixel_to_robot_ray(self):
        correct_ray_cam = Ray(np.array([[0],[0],[0]]),np.array([[0],[0],[1]]),length=1)

        correct_ray = Ray(np.array([[0],[0],[-0.1]]),np.array([[1],[0],[0]]),length=1)

        mid_point = self.camera.get_pixel_through_ray(correct_ray_cam)
        ray_img = self.camera.get_ray_through_image(mid_point)
        ray_rob = self.camera.get_ray_in_robot_frame(ray_img)

        
        # notice low abs_tol, transforms can have large errors in this case
        # we're talking milimiter differences though (2mm max)
        assertRayEquals(self,correct_ray,ray_rob,
            msg="correct: \n {} \n was: \n {}".format(correct_ray,ray_rob),
            abs_tol=2e-3)


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'camera_test',CameraTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest