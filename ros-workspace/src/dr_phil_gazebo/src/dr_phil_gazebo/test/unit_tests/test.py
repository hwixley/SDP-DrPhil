#! /usr/bin/env python3

import unittest
import sys
import time 

PKG='dr_phil_gazebo'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class TestExample(unittest.TestCase):
    def test_hello_world(self):
        print("waiting")
        time.sleep(500)
        print("waited")
        self.assertEqual(1,1)

if __name__ == "__main__":
    import rosunit

    # note rosunit for offline tests
    rosunit.unitrun(PKG,'test_example',TestExample)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest