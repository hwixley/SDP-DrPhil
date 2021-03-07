#! /usr/bin/env python3

import unittest
import sys

PKG='dr_phil_gazebo'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class LidarTest(unittest.TestCase):
    def test_hello_world(self):
        self.assertEqual(1,1)
    
    def test_get_unit_vec_from_dir(self):
        pass

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG,'lidar_test',LidarTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest