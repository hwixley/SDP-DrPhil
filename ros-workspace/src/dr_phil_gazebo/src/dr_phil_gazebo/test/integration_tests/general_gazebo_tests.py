#! /usr/bin/env python3

from time import sleep
import unittest

import sys
import rospy
from rospy.exceptions import ROSException


PKG='dr_phil_gazebo'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest


class JointStatesPublished(unittest.TestCase):

    def runTest(self):
        from sensor_msgs.msg  import JointState

        received = False 

        try:
           rospy.wait_for_message("/joint_states",JointState,timeout=5)
           received = True
        except ROSException as E:
            pass

        self.assertTrue(received,"/joint_states not published")

class TFPublished(unittest.TestCase):

    def runTest(self):
        from tf.msg import tfMessage

        received = False

        try:
            rospy.wait_for_message("/tf",tfMessage,timeout=5)
            received = True

        except ROSException as E:
            pass 

        self.assertTrue(received,"/tf not published")

class SuccRunTestSuite(unittest.TestSuite):
    def __init__(self):
        rospy.init_node('test_node',anonymous=True)

        super(SuccRunTestSuite,self).__init__()
    
        self.addTest(JointStatesPublished())
        self.addTest(TFPublished())

        # we delay tests until gazebo is running, or error out if it's not
        try:
            rospy.wait_for_service("/gazebo/get_link_state",timeout=10)
        except ROSException as E:
            raise Exception("Gazebo has not started successfully, cannot run tests")

if __name__ == "__main__":
    import rostest

    # note rostest for online tests
    rostest.rosrun(PKG,'succ_run_test',"dr_phil_gazebo.test.integration_tests.general_gazebo_tests.SuccRunTestSuite")

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest