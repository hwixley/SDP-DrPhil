#! /usr/bin/env python3

import unittest
import sys
import rospy 
import py_trees
from dr_phil_hardware.behaviour.leafs.ros import ActionClientConnectOnInit
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal
from dr_phil_hardware.test.behaviour.behaviour_test_utils import one_tick_success_leaf_fibonacci

PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class ActionClientConnectOnInitTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node("action_client_connect_on_init_test",anonymous=True)
        self.rate = rospy.Rate(1)
        self.time_start = rospy.get_time()

        
    def test_action_client_no_fail(self):

        root = one_tick_success_leaf_fibonacci("/mock_action_server")
        # tick a bit more than timeout
        while (rospy.get_time() - self.time_start) < 2:
            root.tick_once()
            self.rate.sleep()

        self.assertNotEqual(root.status,py_trees.Status.FAILURE,"Action client failed with a server running after timeout")


    def test_no_action_client_fail(self):
        root = one_tick_success_leaf_fibonacci("/gibberish")

        # tick a bit more than timeout
        while (rospy.get_time() - self.time_start) < 2:
            root.tick_once()
            self.rate.sleep()

        self.assertEqual(root.status,py_trees.Status.FAILURE,"Action client didn't fail without a server after timeout")

    def test_running_before_timeout(self):

        root = one_tick_success_leaf_fibonacci("/gibberish")


        # tick a bit more than timeout
        while (rospy.get_time() - self.time_start) < 0.5:
            root.tick_once()
            self.rate.sleep()

        self.assertEqual(root.status,py_trees.Status.RUNNING,"Action client didn't return running when not connected")


    def test_completion_success(self):

        root = one_tick_success_leaf_fibonacci("/mock_action_server")

        # tick a bit more to let it finish
        while (rospy.get_time() - self.time_start) < 2:
            root.tick_once()
            self.rate.sleep()

        self.assertEqual(root.status,py_trees.Status.SUCCESS,"Action client didn't return running when not connected")
    
    def test_preemption_invalid(self):

        root = one_tick_success_leaf_fibonacci("/mock_action_server")

        # tick once but not enough to let finish
        while (rospy.get_time() - self.time_start) < 0.5:
            root.tick_once()
            self.rate.sleep()

        root.stop()

        self.assertEqual(root.status,py_trees.Status.INVALID,"Action didnt become invalid when pre-empted")

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'action_client_connect_on_init_test',ActionClientConnectOnInitTest)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest