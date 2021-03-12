#! /usr/bin/env python3

import unittest

import py_trees
from dr_phil_hardware.arm_interface.command_arm import ArmCommander, MoveGroup

from dr_phil_hardware.behaviour.leafs.ros import ExecuteMoveItPlan
from dr_phil_hardware.test.behaviour.behaviour_test_utils import tick_untill_not_running
import math
import rospy
from geometry_msgs.msg import Pose 
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Empty
PKG='dr_phil_hardware'
import roslib; roslib.load_manifest(PKG)
# all unit tests are compatibile with ros
# only requirement is that we return results in an appropriate xml format 
# see: http://wiki.ros.org/unittest

class testExecuteMoveitPlan(unittest.TestCase):

    def get_test_instance_tree(self,dwt):
        root = ExecuteMoveItPlan("test",MoveGroup.ARM,distance_waypoint_threshold=dwt,time_threshold_mult=0)
        root.setup(1)
        return root

    def setUp(self):
        rospy.init_node("test_node",anonymous=True)
        rospy.Publisher("reset_joints",Empty,queue_size=10).publish(Empty()) # reset joint states

        py_trees.Blackboard().set(ExecuteMoveItPlan.PLAN_SOURCE,None)

    def test_no_plan_fails(self):
        i = self.get_test_instance_tree(0.01)

        tick_untill_not_running(i,tick_limit=1)

        self.assertIs(i.status,py_trees.Status.FAILURE,"No plan did not fail")
 
    def test_empty_plan_succeeds(self):
        i = self.get_test_instance_tree(0.01)
        py_trees.Blackboard().set(ExecuteMoveItPlan.PLAN_SOURCE,RobotTrajectory())
        tick_untill_not_running(i,tick_limit=20)

        self.assertIs(i.status,py_trees.Status.SUCCESS,"empty plan did not succeed")

    def test_valid_plan_succeeds(self):
        i = self.get_test_instance_tree(0.01)

        ac = ArmCommander()
        # plan to start pose
        ac.set_pose_target(MoveGroup.ARM,ac.get_current_pose(MoveGroup.ARM))

        (_,plan,_,_) = ac.plan(MoveGroup.ARM)

        py_trees.Blackboard().set(ExecuteMoveItPlan.PLAN_SOURCE,plan)
        tick_untill_not_running(i,tick_limit=20)

        self.assertIs(i.status,py_trees.Status.SUCCESS,"valid plan did not succeed")

    def test_valid_plan_executes_in_time(self):
        i = self.get_test_instance_tree(0.01)

        ac = ArmCommander()
        # plan to start pose
        pose2 = ac.get_current_pose(MoveGroup.ARM).pose
        pose2.position.z -= 0.1
        ac.set_pose_target(MoveGroup.ARM,pose2)
        ac.set_curr_state_as_start_state(MoveGroup.ARM)
        ac.set_max_velocity_scaling_factor(MoveGroup.ARM,0.1)
        ac.set_goal_tolerance(MoveGroup.ARM,0.1)
        (_,plan,_,_) = ac.plan(MoveGroup.ARM)
        plan.joint_trajectory.header.stamp = rospy.Time.now()

        time_end = plan.joint_trajectory.header.stamp + plan.joint_trajectory.points[-1].time_from_start
        correct_length = plan.joint_trajectory.points[-1].time_from_start

        py_trees.Blackboard().set(ExecuteMoveItPlan.PLAN_SOURCE,plan)
        tick_untill_not_running(i,tick_limit=math.inf,freq=100)

        time_ended = rospy.Time.now()
        time_taken = time_ended - plan.joint_trajectory.header.stamp 

        self.assertIs(i.status,py_trees.Status.SUCCESS,"valid plan did not succeed")
        self.assertAlmostEqual(float(time_end.to_sec()),float(time_ended.to_sec()),
                            None,
                            delta=0.1,
                            msg="plan ended in time differing: {} seconds from time it was supposed to end. Took {} seconds, was supposed to take: {}".format((time_end - time_ended).to_sec(),
                                                                                                                                                                    time_taken.to_sec(),correct_length.to_sec()))
    def test_valid_plan_fails_on_obstacle(self):
        i = self.get_test_instance_tree(0.02)

        ac = ArmCommander()
        # plan to start pose
        pose2 = ac.get_current_pose(MoveGroup.ARM).pose
        pose2.position.z -= 0.1
        ac.set_pose_target(MoveGroup.ARM,pose2)
        ac.set_curr_state_as_start_state(MoveGroup.ARM)
        ac.set_max_velocity_scaling_factor(MoveGroup.ARM,0.1)
        ac.set_goal_tolerance(MoveGroup.ARM,0.1)
        (_,plan,_,_) = ac.plan(MoveGroup.ARM)
        plan.joint_trajectory.header.stamp = rospy.Time.now()

        # prevent plan from execution
        rospy.Publisher("/simulate_obstacle",Empty,queue_size=10).publish(Empty())

        py_trees.Blackboard().set(ExecuteMoveItPlan.PLAN_SOURCE,plan)
        tick_untill_not_running(i,tick_limit=math.inf,freq=100)

        self.assertIs(i.status,py_trees.Status.FAILURE,msg="Plan executed succesfully even though constraint prevented it from being physically executed")
if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG,'test_execute_moveit_plan',testExecuteMoveitPlan)

# to run me, individually: just execute me ./test.py 
# to run all the tests listed in CMakeLists.txt: catkin_make run_tests_dr_phil_hardware_rostest