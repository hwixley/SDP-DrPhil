#!/usr/bin/env python3

from enum import Enum

import moveit_commander
import sys
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
import rospy 
import copy
from moveit_msgs.msg import MoveItErrorCodes,DisplayTrajectory,RobotTrajectory,DisplayRobotState,RobotState
from moveit_msgs.srv import GetPositionFK,GetPositionFKResponse
from geometry_msgs.msg import Pose,PoseStamped
from std_msgs.msg import Header
from typing import Tuple,List, final
import numpy as np

class MoveGroup(str,Enum):
    ARM="arm"


class ArmCommander():
    """ Python API for the moveit interface specific to dr_phil, 
    requires to be run inside a node 
    """

    class __ArmCommander():


        def __init__(self) -> None:
            """ 
                Args:
            """
            moveit_commander.roscpp_initialize([])
            self.robot : RobotCommander = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_groups = {}

            for mg in self.robot.get_group_names():
                new_mg:MoveGroupCommander = moveit_commander.MoveGroupCommander(mg)
                self.move_groups[mg] = new_mg
                self.set_workspace(mg,[-1,-1,1,1])
                self.set_planner_id(mg,"SPARS")
                new_mg.set_max_velocity_scaling_factor(1)
                new_mg.set_num_planning_attempts(10)
                new_mg.set_planning_time(10)
                new_mg.set_start_state_to_current_state()
                new_mg.allow_replanning(True)
                new_mg.set_goal_orientation_tolerance(1.3)
                new_mg.set_goal_joint_tolerance(0.5)
                new_mg.set_goal_position_tolerance(0.05)
                
            self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        DisplayTrajectory,
                                                        queue_size=20)
            self.display_state_publisher = rospy.Publisher('/display_robot_state',
                                                            DisplayRobotState,
                                                            queue_size=20)

        def clear_state(self,group : MoveGroup):
            mg = self.__get_move_group(group)
            mg.clear_pose_targets()
            mg.clear_path_constraints()
            # mg.clear_trajectory_constraints()

        def __get_move_group(self,group : MoveGroup) -> MoveGroupCommander:
            return self.move_groups[group]

        def __visualise_state(self,plan : RobotTrajectory):
            displayTraj = DisplayTrajectory()
            displayTraj.trajectory_start = self.robot.get_current_state()
            displayTraj.trajectory.append(plan)

            self.display_trajectory_publisher.publish(displayTraj)

            displayState = DisplayRobotState()
            displayState.state = self.robot.get_current_state()

            self.display_state_publisher.publish(displayState)

        def execute_plan(self,group : MoveGroup, plan : RobotTrajectory, blocking=False):
            """ executes a given plan, will block if async is true """

            mg = self.__get_move_group(group)
            self.__visualise_state(plan)
            mg.execute(plan,wait=blocking)

        def plan(self,group: MoveGroup) -> Tuple[bool,RobotTrajectory,float,MoveItErrorCodes]:
            mg = self.__get_move_group(group)
            (b,rt,pt,err) = mg.plan()
            rt.joint_trajectory.header.stamp = rospy.Time.now()
            return (b,rt,pt,err)
        
        def go(self,group : MoveGroup,blocking=False):
            mg = self.__get_move_group(group)
            self.__visualise_state(mg.plan()[1])
            mg.go(wait=blocking)

        def plan_smooth_path(self,group: MoveGroup,waypoints : List[Pose] ) -> Tuple[RobotTrajectory,float]:
            """ returns plan and how fraction of how much it follows the path between the waypoints """
            mg = self.__get_move_group(group)
            mg.set_start_state_to_current_state()
            mg.clear_pose_targets()

            curr_pose = mg.get_current_pose().pose

            final_waypoints = [curr_pose] + waypoints
            
            (plan,fraction) = mg.compute_cartesian_path(final_waypoints,
                                                        0.01,
                                                        0) # TODO: check this value, zero jump threshold is bad for hardware
            plan.joint_trajectory.header.stamp = rospy.Time.now()

            return plan,fraction

        def set_workspace(self,group:MoveGroup,ws :list) -> None :
            mg = self.__get_move_group(group)
            mg.set_workspace(ws)

        def get_planning_frame(self,group:MoveGroup) -> str : 
            mg =  self.__get_move_group(group)
            return mg.get_planning_frame()

        def get_pose_planning_frame(self,group:MoveGroup) -> str : 
            mg =  self.__get_move_group(group)
            return mg.get_pose_reference_frame()

        def set_pose_planning_frame(self,group:MoveGroup, pf:str): 
            mg =  self.__get_move_group(group)
            return mg.set_pose_reference_frame(pf)

        def get_end_effector_link(self,group:MoveGroup) -> str :
            """ can be empty string """ 
            mg =  self.__get_move_group(group)
            return mg.get_end_effector_link()
            
        def get_current_joint_states(self,group:MoveGroup) -> list:
            mg = self.__get_move_group(group)
            return mg.get_current_joint_values()

        def get_current_pose(self,group:MoveGroup) -> PoseStamped:
            mg = self.__get_move_group(group)
            return copy.deepcopy(mg.get_current_pose())

        def set_pose_target(self,group:MoveGroup,pose : Pose) -> PoseStamped:
            mg = self.__get_move_group(group)
            mg.set_pose_target(pose)

        def get_random_pose(self,group:MoveGroup) -> PoseStamped:
            mg = self.__get_move_group(group)
            return mg.get_random_pose()

        def set_max_velocity_scaling_factor(self,group:MoveGroup,v) -> None:
            mg = self.__get_move_group(group)
            mg.set_max_velocity_scaling_factor(v)

        def set_goal_tolerance(self,group:MoveGroup,v) -> None:
            mg = self.__get_move_group(group)
            mg.set_goal_tolerance(v)
        
        def set_orientation_goal_tolerance(self,group:MoveGroup,v) -> None:
            mg = self.__get_move_group(group)
            mg.set_goal_orientation_tolerance(v)
        
        def set_joint_goal_tolerance(self,group:MoveGroup,v) -> None:
            mg = self.__get_move_group(group)
            mg.set_goal_joint_tolerance(v)

        def set_position_goal_tolerance(self,group: MoveGroup,v) -> None:
            mg = self.__get_move_group(group)
            mg.set_goal_position_tolerance(v)

        def get_planner_id(self,group:MoveGroup) -> str:
            mg = self.__get_move_group(group)
            return mg.get_planner_id()

        def set_planner_id(self,group:MoveGroup, planner : str) -> str:
            mg = self.__get_move_group(group)
            return mg.set_planner_id(planner)

        def halt(self,group:MoveGroup) -> None:
            mg = self.__get_move_group(group)
            mg.stop()

        def set_curr_state_as_start_state(self,group:MoveGroup) -> None:
            mg = self.__get_move_group(group)
            mg.set_start_state_to_current_state()

        def get_jacobian(self,group:MoveGroup,joint_vals=list):
            mg = self.__get_move_group(group)

            J = mg.get_jacobian_matrix(joint_vals)

            return np.reshape(np.asfarray(J),newshape=(6,len(joint_vals)))

        def compute_fk(self,link_names : List[str],output_frame : str, robotState : RobotState) -> Tuple[List[PoseStamped],List[str],MoveItErrorCodes]:
            header = Header()
            header.frame_id = output_frame
            header.stamp = rospy.Time.now()

            response : GetPositionFKResponse =  rospy.ServiceProxy("/compute_fk",GetPositionFK)(header,link_names,robotState)
            return response.pose_stamped,response.fk_link_names,response.error_code

        def get_current_robot_state(self) -> RobotState:
            return self.robot.get_current_state()
        def get_joints(self,group:MoveGroup) -> str:
            return self.__get_move_group(group).get_active_joints()

    instance = None

    def __init__(self):
        if not ArmCommander.instance:
            ArmCommander.instance = ArmCommander.__ArmCommander()
        else:
            pass

    def __getattribute__(self,name):
        if name == "instance":
            return ArmCommander.instance
        else:
            return getattr(self.instance, name)

    # --------------------
    # JUST FOR TYPE HINTS
    # --------------------
    def plan_smooth_path(self,group: MoveGroup,waypoints : List[Pose] ) -> Tuple[RobotTrajectory,float]:assert(False)
    def execute_plan(self,group:MoveGroup,plan : RobotTrajectory,blocking=False): assert(False)
    def get_current_pose(self,group:MoveGroup) -> PoseStamped: assert(False)
    def get_random_pose(self,group:MoveGroup) -> PoseStamped: assert(False)
    def set_max_velocity_scaling_factor(self,group:MoveGroup,v) -> None :assert(False)
    def set_goal_tolerance(self,group:MoveGroup,v) -> None: assert(False)
    def set_orientation_goal_tolerance(self,group:MoveGroup,v) -> None: assert(False)
    def set_joint_goal_tolerance(self,group:MoveGroup,v) -> None: assert(False)
    def set_position_goal_tolerance(self,group: MoveGroup,v) -> None: assert(False)
    def get_planner_id(self,group:MoveGroup) -> str: assert(False)
    def set_planner_id(self,group:MoveGroup, planner : str) -> str: assert(False)
    def go(self,group : MoveGroup,blocking=False): assert(False)
    def set_pose_target(self,group:MoveGroup,pose : Pose) -> PoseStamped: assert(False)
    def get_planning_frame(self,group:MoveGroup) -> str : assert(False)
    def clear_state(self,group : MoveGroup): assert(False)
    def plan(self,group: MoveGroup) -> Tuple[bool,RobotTrajectory,float,MoveItErrorCodes]: assert(False)
    def set_pose_planning_frame(self,group:MoveGroup, pf:str): assert(False) 
    def halt(self,group:MoveGroup) -> None: assert(False)
    def set_curr_state_as_start_state(self,group:MoveGroup) -> None:assert(False)
    def get_jacobian(self,group:MoveGroup,joint_vals:list): assert(False)
    def compute_fk(self,link_names : List[str],output_frame : str, robotState : RobotState) -> Tuple[List[PoseStamped],List[str],MoveItErrorCodes]: assert(False)
    def get_current_robot_state(self) -> RobotState: assert(False)
    def get_end_effector_link(self,group:MoveGroup) -> str : assert(False)
    def get_joints(self,group:MoveGroup) -> List[str]: assert(False)

# if started as a node it will perform a test trajectory
if __name__ == "__main__":
    rospy.init_node("test_trajectory",anonymous=True)

    a = ArmCommander()
    a.clear_state(MoveGroup.ARM)

    print(a.get_planning_frame(MoveGroup.ARM))
    a.set_max_velocity_scaling_factor(MoveGroup.ARM,1)
    # a.set_goal_tolerance(MoveGroup.ARM,0.05) # to 1 cm accuracy
    a.set_planner_id(MoveGroup.ARM,"SPARS")

    pose2 = a.get_current_pose(MoveGroup.ARM).pose
    pose2.position.z -= 0.2
    waypoints = [pose2]

    (plan,_) = a.plan_smooth_path(MoveGroup.ARM,waypoints)

    a.execute_plan(MoveGroup.ARM,plan)