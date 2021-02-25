#! /usr/bin/env python3

from __future__ import absolute_import
import unittest
import sys

PKG=u'dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class TestExample(unittest.TestCase):
    def test_hello_world(self):
        self.assertEqual(1,1)

if __name__ == u"__main__":
    import rosunit
    rosunit.unitrun(PKG,u'test_example',TestExample)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest