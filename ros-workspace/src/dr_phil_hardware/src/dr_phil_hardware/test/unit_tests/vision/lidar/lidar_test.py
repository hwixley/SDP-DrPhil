#! /usr/bin/env python3

import unittest
import sys
import rospy 
from sensor_msgs.msg import CameraInfo
from dr_phil_hardware.vision.lidar import Lidar
import tf 
from tf2_msgs.msg import TFMessage
from dr_phil_hardware.test.test_utils.test_utils import assertRayEquals 
from dr_phil_hardware.vision.ray import Ray
import numpy as np


PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class LidarTest(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("lidar_test",anonymous=True)


        self.robot_frame = "/base_link"
        self.lidar_frame = "/base_scan"

        # setup the lidar and transforms
        transform_listener = tf.TransformListener()
        transformerRos = tf.TransformerROS()

        self.lidar = Lidar()

        # this will block
        try:
            rospy.wait_for_message("/tf_static",TFMessage,timeout=rospy.Duration(5))
        except:
            self.fail("Could not setup transforms")


        # wait for transform to become available
        transform_listener.waitForTransform(
            self.lidar_frame,
            self.robot_frame,
            rospy.Time.now(),
            timeout=rospy.Duration(2))

        # get transform
        (trans,rot) = transform_listener.lookupTransform(
            self.lidar_frame,
            self.robot_frame,
            rospy.Time.now())
        
        try:
            self.rob2lidar = transformerRos.fromTranslationRotation(trans,rot)
            self.lidar.setup_transform(self.rob2lidar)

        except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
            self.fail("Could not setup transforms")

    def test_x_unit_robot_ray_to_lidar(self):

        # vector pointing forward in robot space
        correct_ray_robot = Ray(np.array([[0],[0],[0]]),np.array([[1],[0],[0]]),length=1)
        # correct ray originates -0.2m but with same direction
        correct_ray_lidar = Ray(np.array([[0],[0],[-0.2]]),np.array([[1],[0],[0]]),length=1)

        #notice inversion of pixels
        ray_lidar = self.lidar.get_ray_in_lidar_frame(correct_ray_robot)


        assertRayEquals(self,correct_ray_lidar,ray_lidar,
            msg="correct: \n {} \n was: \n {}".format(correct_ray_lidar,ray_lidar))

    

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'lidar_test',LidarTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest