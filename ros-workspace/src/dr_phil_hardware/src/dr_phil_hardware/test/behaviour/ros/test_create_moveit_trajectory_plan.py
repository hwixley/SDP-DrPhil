#! /usr/bin/env python3

import unittest

import py_trees
from dr_phil_hardware.arm_interface.command_arm import MoveGroup

from dr_phil_hardware.behaviour.leafs.ros import CreateMoveitTrajectoryPlan
from dr_phil_hardware.test.behaviour.behaviour_test_utils import tick_untill_not_running

import rospy
from geometry_msgs.msg import Pose,PoseArray
from moveit_msgs.msg import RobotTrajectory

PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class testCreateMoveitTrajectoryPlan(unittest.TestCase):

    def get_test_instance_tree(self,mg,fraction):
        root = CreateMoveitTrajectoryPlan("test",move_group=mg,fraction_threshold=fraction)
        root.setup(20)
        return root

    def setUp(self):
        rospy.init_node("test_node",anonymous=True)
        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,None)
        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.PLAN_TARGET,None)

    def test_any_plan_success(self):
        i = self.get_test_instance_tree(MoveGroup.ARM,0)
        ticks = 100
        
        p2 = Pose()
        p2.position.x = 1
        pa = PoseArray()
        pa.poses.append(p2)
        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,pa)

        tick_untill_not_running(i,ticks)

        self.assertIs(i.status,py_trees.Status.SUCCESS,"Any plan did not succeed after {} ticks".format(ticks))

    def test_any_plan_correct_plan_type(self):
        i = self.get_test_instance_tree(MoveGroup.ARM,0)
        ticks = 100
        
        p2 = Pose()
        p2.position.x = 1
        pa = PoseArray()
        pa.poses.append(p2)

        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,pa)

        tick_untill_not_running(i,ticks)


        self.assertIsNotNone(py_trees.Blackboard().get(CreateMoveitTrajectoryPlan.PLAN_TARGET),"Plan was not put on the blackboard, blackboard: {}".format(py_trees.Blackboard()))

        self.assertIsInstance(py_trees.Blackboard().get(CreateMoveitTrajectoryPlan.PLAN_TARGET),RobotTrajectory,
            "Plan was not the right message type on the blackboard")

    def test_no_input_fails(self):
        i = self.get_test_instance_tree(MoveGroup.ARM,0)
        ticks = 1
        tick_untill_not_running(i,ticks)
        self.assertIs(i.status,py_trees.Status.FAILURE,"Any plan did not fail after {} ticks".format(ticks))

    def test_impossible_plan_input_fails(self):
        i = self.get_test_instance_tree(MoveGroup.ARM,0.5001) # only half of the plan will be possible (i.e. start pose)
        ticks = 100
        p2 = Pose()
        p2.position.x = float("inf")
        p2.position.y = float("inf")
        p2.position.z = float("inf")
        pa = PoseArray()
        pa.poses.append(p2)
        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,pa)

        tick_untill_not_running(i,ticks)
        
        self.assertIs(i.status,py_trees.Status.FAILURE,"impossible plan did not fail after {} ticks".format(ticks))


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'test_create_moveit_trajectory_plan',testCreateMoveitTrajectoryPlan)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest