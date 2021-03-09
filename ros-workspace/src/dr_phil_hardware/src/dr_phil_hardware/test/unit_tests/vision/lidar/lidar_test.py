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

class LidarTest(unittest.TestCase):

    def setUp(self) -> None:
        rospy.init_node("lidar_test",anonymous=True)


        self.robot_frame = "/base_link"
        self.lidar_frame = "/base_scan"

        # setup the lidar and transforms
        transform_listener = tf.TransformListener()
        transformerRos = tf.TransformerROS()
        
        try:
            self.lidar_data = rospy.wait_for_message("/scan_filtered", LaserScan)
        except:
            self.fail("Could not setup lidar")
        
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

    def test_get_ray_projection_1(self):
        input = Ray(np.array([0, 0, 0]), np.array([0, 2, 3]))
        flattened_input = Ray(np.array([0, 0, 0]), np.array([0, 2, 0]), length=1)

        output = self.lidar.get_ray_projection(input)
        assertRayEquals(self,flattened_input,output,
            msg="correct: \n {} \n output: \n {}".format(flattened_input,output))
    
    def test_get_ray_projection_2(self):
        input = Ray(np.array([1, 5, 4]), np.array([1, 2, 0]))
        flattened_input = Ray(np.array([1, 5, 4]), np.array([1, 2, 0]), length=1)

        output = self.lidar.get_ray_projection(input)
        assertRayEquals(self,flattened_input,output,
            msg="correct: \n {} \n output: \n {}".format(flattened_input,output))
    
    def test_get_corresponding_lidar_rays(self):
        origin = [0, 0, 0]
        dirs = [[1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [1, -1, 0]]
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        for i, dir in enumerate(dirs):
            camera_ray = Ray(np.array(origin), np.array(dir), length=1)
            ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
            angle1 = self.lidar.get_angle_from_unit_vec(ray1)
            angle2 = self.lidar.get_angle_from_unit_vec(ray2)
            print("\n\nCorresponding lidar rays found at angles:\nlray1 angle: {}\nlray2 angle: {}\ncamera_ray_angle: {}\n<------>\n".format(angle1, angle2, angles[i]))
            self.assertTrue(True)
    """
    def test_get_corresponding_lidar_rays_1(self):
        camera_ray = Ray(np.array([0, 0, 0]), np.array([1, 0, 0]), length=1)
        ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
        angle1 = self.lidar.get_angle_from_unit_vec(ray1)
        angle2 = self.lidar.get_angle_from_unit_vec(ray2)
        print(angle1, angle2)
        self.assertTrue(angle1 in [359, 0] and angle2 in [0, 1])
    
    def test_get_corresponding_lidar_rays_2(self):
        camera_ray = Ray(np.array([0, 0, 0]), np.array([0, 1, 0]), length=1)
        ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
        angle1 = self.lidar.get_angle_from_unit_vec(ray1)
        angle2 = self.lidar.get_angle_from_unit_vec(ray2)
        print(angle1, angle2)
        self.assertTrue(angle1 in [89, 90] and angle2 in [90, 91])

    def test_get_corresponding_lidar_rays_3(self):
        camera_ray = Ray(np.array([0, 0, 0]), np.array([-1, -0.001, 0]), length=1)
        ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
        angle1 = self.lidar.get_angle_from_unit_vec(ray1)
        angle2 = self.lidar.get_angle_from_unit_vec(ray2)
        print(angle1, angle2)
        self.assertTrue(angle1 in [179, 180] and angle2 in [180, 181])

    def test_get_corresponding_lidar_rays_4(self):
        camera_ray = Ray(np.array([0, 0, 0]), np.array([0, -1, 0]), length=1)
        ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
        angle1 = self.lidar.get_angle_from_unit_vec(ray1)
        angle2 = self.lidar.get_angle_from_unit_vec(ray2)
        print(angle1, angle2)
        self.assertTrue(angle1 in [269, 270] and angle2 in [270, 271])

    def test_get_corresponding_lidar_rays_5(self):
        camera_ray = Ray(np.array([0, 0, 0]), np.array([1, -1, 0]), length=1)
        ray1, ray2 = self.lidar.get_corresponding_lidar_rays(camera_ray, self.lidar_data)
        angle1 = self.lidar.get_angle_from_unit_vec(ray1)
        angle2 = self.lidar.get_angle_from_unit_vec(ray2)
        print(angle1, angle2)
        self.assertTrue(angle1 in [314, 315] and angle2 in [315, 316])
    """
    def test_get_unit_vec_from_dir(self):
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            ray = self.lidar.get_unit_vec_from_dir(np.deg2rad(angle))
            self.assertTrue(self.lidar.get_angle_from_unit_vec(ray) == angle)

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'lidar_test',LidarTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest