#!/usr/bin/env python3

from py_trees import blackboard
import py_trees_ros
from py_trees.blackboard import Blackboard
from dr_phil_hardware.arm_interface.command_arm import ArmCommander, MoveGroup
import py_trees
from dr_phil_hardware.behaviour.leafs.ros import CallService, CreateMoveItMove, DynamicReconfigure, PublishTopic,RunRos,MessageChanged,ActionClientConnectOnInit,CreateMoveitTrajectoryPlan,ExecuteMoveItPlan
from dr_phil_hardware.behaviour.leafs.general import ClosestObstacle, Lambda,SetBlackboardVariableCustom,CheckFileExists
from dr_phil_hardware.behaviour.decorators import HoldStateForDuration
import operator
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
import os
from std_msgs.msg import Float64MultiArray
from py_trees.common import BlackBoxLevel, Status
import rospy 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_srvs.srv import Empty,EmptyRequest
import tf.transformations as t 
from dr_phil_hardware.vision.utils import quat_from_yaw
import numpy as np
from dr_phil_hardware.srv import GenerateTarget,GenerateTargetRequest,GenerateTargetResponse
import math 
from move_base.cfg import MoveBaseConfig

def create_exploration_completed_check(duration=60):
    """ creates subtree which returns SUCCESS if no data has been received from the exploration nodes for the given duration.
        Returns RUNNING if data comes through in this time. 

        Args:
            duration: the time after which if no data is received on `/explore/frontiers` to return SUCCESS
    """

    fail_no_new_data = MessageChanged(name="CmdVelChanged?",
                                        topic_name="/cmd_vel",
                                        topic_type=Twist)
    

    hold_state = HoldStateForDuration(fail_no_new_data,"HoldFailureIgnoreRunning",duration,state=py_trees.common.Status.FAILURE,ignore_running=True)

    f2rHold_state = py_trees.decorators.FailureIsRunning(hold_state)
    return f2rHold_state




def create_set_positions_arm(parameters,name="positionArm"):
    
    # clean inputs
    lower_limits = [0, -1.57, -1.57, -1.57, -1.57, -1]
    upper_limits = [0, 1.57, 1.57, 1.57, 1.57, 1.57]
    clean_lower = max(lower_limits,parameters)
    clean_upper = min(clean_lower,upper_limits)
    parameters = list(clean_upper)

    # create message
    positions = Float64MultiArray()
    positions.data = parameters 

    position_arm = PublishTopic(
        name=name,
        msg=positions,
        msg_type=Float64MultiArray,
        topic="/joint_trajectory_point",
        success_after_n_publishes=5,
        queue_size=10
    )
    

    return position_arm

def create_explore_frontier_and_save_map(map_path=None,timeout=120,no_data_timeout=20):
    """ creates subtree which executes frontier exploration, generates a map and saves it to the given map_name 
        Args:
            map_path: the absolute path to where to save the map
            timeout: the time after which to stop mapping overall
            no_data_timeout: the time after which if we receive no new frontier data to consider the map to be complete
    
    """
    sequence = py_trees.composites.Sequence()

    neutral_pos = [0,0,-1.5,1.5,-1.5,0]
    resetArmPosition = py_trees.decorators.OneShot(
        create_set_positions_arm(neutral_pos,name="clearArmPosition"))

    exploration_kill_key = "/explore/kill"

    # won't return success until exploration is killed and returns success
    parallel_process = py_trees.composites.Parallel()

    startExplorationNodes = RunRos(name="runExplorationNodes",
        blackboard_alive_key="/explore/alive",
        blackboard_kill_trigger_key=exploration_kill_key,
        launch_file="explore_task.launch",
        package="dr_phil_hardware"
    )


    exitSequence = py_trees.composites.Sequence()

    frontierEmptyCheck = create_exploration_completed_check(duration=no_data_timeout)

    map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"map")if map_path is None else map_path  

    saveMap = create_save_map(map_path)

    killExplorationNodes = py_trees.blackboard.SetBlackboardVariable(name="SetKillTrigger",
        variable_name=exploration_kill_key,
        variable_value="night night")

    haltMsg = Twist()
    haltMsg.linear.x = 0
    haltMsg.angular.z = 0 

    halt = PublishTopic(
        name="haltCmdVel",
        msg=haltMsg,
        msg_type=Twist,
        topic="/cmd_vel",
        success_after_n_publishes=2
    )
    
    wait = py_trees.timers.Timer(duration=5)

    exitSequence.add_children([frontierEmptyCheck,saveMap,killExplorationNodes,halt,wait])
    
    parallel_process.add_children([startExplorationNodes,exitSequence])

    sequence.add_children([resetArmPosition,parallel_process])

    timeoutN = py_trees.decorators.Timeout(sequence,duration=timeout)

    return timeoutN



    
def create_disinfect_doors_in_map(handle_pose_src,spray_path_src,map_path=None,distance_from_door=0.1,name="disinfectDoors"):
    """[summary]

    Args:
        handle_pose_src (`str->Pose`): 
        spray_path_src (`str->PoseArray`):
        map_path ([type], optional): [description]. path to file without extension.
        name (str, optional): [description]. Defaults to "disinfectDoors".
    """

    assert(not ".yaml" in map_path  or not ".pgm" in map_path)

    bb2bufferPose = "{}/handle_pose".format(name)
    bb2bufferSpray = "{}/spray_poses".format(name)

    # ------ start nodes ----------
    parallel = py_trees.composites.Parallel("disinfectDoors")

    load_map = create_load_map(map_path=map_path + ".yaml")

    start_disinfection_nodes = RunRos(name="runDisinfectionNodes",
        blackboard_alive_key="/disinfect/alive",
        blackboard_kill_trigger_key="/disinfect/kill",
        launch_file="disinfect_task.launch",
        package="dr_phil_hardware"
    )
    start_disinfection_nodes.blackbox_level = BlackBoxLevel.DETAIL

    # ------ do some movement ---------

    sequence = py_trees.composites.Sequence(name="mainSequence")
    sequence.blackbox_level = BlackBoxLevel.BIG_PICTURE

    wait = py_trees.timers.Timer(duration=8)

    reset_arm = create_set_positions_arm([0,0,-1.5,1.5,-1.5,0],name="resetArm")

    localize = create_localize_robot()
    localize.blackbox_level = BlackBoxLevel.COMPONENT


    def snapshot_targets(snapshot=False):
        handle = py_trees.Blackboard().get(handle_pose_src)
        spray_path = py_trees.Blackboard().get(spray_path_src)

        if handle is not None and spray_path is not None:
            # save
            if snapshot:
                py_trees.Blackboard().set(bb2bufferSpray,spray_path)
                py_trees.Blackboard().set(bb2bufferPose,handle)
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING
    wait = py_trees.timers.Timer(name="wait",duration=2)

    await_targets = Lambda("awaitTargets",snapshot_targets)
    
    wait_2 = py_trees.timers.Timer(name="wait",duration=8)

    snapshot_target = Lambda("snapshotTargets",lambda:snapshot_targets(snapshot=True))

    def get_pose():
        ps = PoseStamped()
        ps.pose = Blackboard().get(bb2bufferPose)
        ps.header.frame_id = "base_link"
        return ps 

    def get_spray():
        s = Blackboard().get(bb2bufferSpray)
        return s

    publish_vis_h = PublishTopic("publishHandle",get_pose,PoseStamped,"/vis/handle_target",success_on_publish=True)
    publish_vis_s = PublishTopic("publishSprayTargets",get_spray,PoseArray,"/vis/spray_targets",success_on_publish=True)
    
    move_to_door = create_move_in_front_of_door(bb2bufferPose,bb2bufferSpray,distance_from_door=distance_from_door,name="moveToDoor")

    
    execute_spray_sequence = create_execute_spray_trajectory(bb2bufferSpray)

    sequence.add_children([wait,reset_arm,localize,await_targets,wait_2,snapshot_target,publish_vis_s,publish_vis_h,move_to_door,execute_spray_sequence])

    parallel.add_children([load_map,start_disinfection_nodes,sequence])

    return parallel


def create_load_map(map_path=None):
    """ Args: 
            map_path: the path to the map with .yaml extension """

    map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"map")if map_path is None else map_path  

    loadMap = RunRos(name="loadMap",
        blackboard_alive_key="map/alive",
        blackboard_kill_trigger_key="map/kill",
        node_file="map_server",
        package="map_server",
        args=[map_path]) 

    return loadMap

def create_save_map(map_path=None):
    """ Args: 
            map_path: the path to the map without extension """

    map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"map")if map_path is None else map_path  
    
    saveMap = RunRos(name="runSaveMap",
        blackboard_alive_key="map/alive",
        blackboard_kill_trigger_key="map/kill",
        node_file="map_saver",
        package="map_server",
        args=["-f",map_path],
        success_on_non_error_exit=True) 


    return saveMap

def create_idle(): 
    idle = py_trees.behaviours.Running(name="Idle")

    return idle




def create_face_closest_obstacle(min_distance = 0.5,face_angle=0): 
    """     
    returns behaviour sub-tree which instructs the robot to turn towards and travel to the nearest obstacle within distance of min_distance """    

    # angle opposite of the face angle
    opposite_angle = (face_angle + 180) % 360

    root = py_trees.composites.Sequence()

    checkBB = py_trees.blackboard.CheckBlackboardVariable(
        name="checkBB",
        variable_name="scan")

    sequence = py_trees.composites.Sequence()
    f2r = py_trees.decorators.FailureIsRunning(sequence)

    closestObstacle = ClosestObstacle(name="closestObstacle2BB")
    
    selector = py_trees.Selector()

    checkFacingObstacle = py_trees.blackboard.CheckBlackboardVariable(
        name="checkFacingObstacleBB",
        variable_name="closest_obstacle/angle",
        comparison_operator=operator.eq,
        expected_value=face_angle)


    parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

    selectorTurnRight = py_trees.composites.Sequence(name="Selector Turn Right")

    checkObstacleToRight = py_trees.blackboard.CheckBlackboardVariable(
        name="checkObstacleOnRight",
        variable_name="closest_obstacle/angle",
        comparison_operator=operator.gt,
        expected_value=opposite_angle)

    turnRightMsg = Twist()
    turnRightMsg.linear.x = 0
    turnRightMsg.angular.z = -0.1 
    turnRight = PublishTopic(
        name="turnRight",
        msg=turnRightMsg,
        msg_type=Twist,
        topic="/cmd_vel",
    )

    selectorTurnRight.add_children([checkObstacleToRight,turnRight])
    
    selectorTurnLeft = py_trees.composites.Sequence(name="Selector Turn Left")

    checkObstacleToLeft = py_trees.blackboard.CheckBlackboardVariable(
        name="checkObstacleOnLeft",
        variable_name="closest_obstacle/angle",
        comparison_operator=operator.lt,
        expected_value=opposite_angle)

    turnLeftMsg = Twist()
    turnLeftMsg.linear.x = 0
    turnLeftMsg.angular.z = 0.1 
    turnLeft = PublishTopic(
        name="turnLeft",
        msg=turnLeftMsg,
        msg_type=Twist,
        topic="/cmd_vel",
    )
    selectorTurnLeft.add_children([checkObstacleToLeft,turnLeft])

    parallel.add_children([selectorTurnLeft,selectorTurnRight])

    selector.add_children([checkFacingObstacle,parallel])
    

    sequence.add_children([closestObstacle,selector])

    root.add_children([checkBB,f2r])

    
    return root


def create_move_base_to_reach_pose(target_pose_src,name="reachTargetPos"):
    """ reaches first target in the given pose array on the blackboard """


    def get_pose():
        return py_trees.Blackboard().get(target_pose_src)

    move_base = ActionClientConnectOnInit(name,
                            MoveBaseAction,
                            get_pose,
                            action_namespace="/move_base",
                            override_feedback_message_on_running="moving to reach target")

    return move_base

def create_try_various_arm_plans(pose_frame):
    plan = py_trees.composites.Selector()

    create_trajectory_plan = CreateMoveitTrajectoryPlan("planTrajectory",
        MoveGroup.ARM,0.9,
        pose_frame=pose_frame,
        pose_target_include_only_idxs=[0],
        )

    create_lenient_trajectory_plan = CreateMoveitTrajectoryPlan("planTrajectoryLenient",
        MoveGroup.ARM,0.5,
        pose_frame=pose_frame,
        pose_target_include_only_idxs=[0],
        )

    create_p2p_plan = CreateMoveItMove("planP2PMove",
        MoveGroup.ARM,
        pose_frame=pose_frame,
        goal_tolerance=0.01,
        )

    create_p2p_plan_lenient = CreateMoveItMove("planP2PMoveLenient",
        MoveGroup.ARM,
        pose_frame=pose_frame,
        goal_tolerance=0.5,
        )

    plan.add_children([create_trajectory_plan,create_lenient_trajectory_plan,create_p2p_plan,create_p2p_plan_lenient])
    return plan

def create_execute_spray_trajectory(target_pose_src):

    spray = py_trees.Sequence("executeSprayTrajectory")

    def process_pose():
        return py_trees.Blackboard().get(target_pose_src)
        
    move_target = SetBlackboardVariableCustom(
            variable_name=CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,
            variable_value=process_pose,
            name="moveTarget2BB")
    
    traj_sequence =  py_trees.composites.Sequence()

    def check():
        poses : PoseArray = py_trees.Blackboard().get(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE)
        if len(poses.poses) != 0:
            return py_trees.Status.SUCCESS
        else: # not completed
            return py_trees.Status.FAILURE 

    check_not_completed = Lambda("check_not_completed",check)

    def get_spray():
        s = Blackboard().get(target_pose_src)
        return s

    publish_vis_s = PublishTopic("publishSprayTargets",get_spray,PoseArray,"/vis/spray_targets",success_on_publish=True)

    
    plan = create_try_various_arm_plans("base_link")
    plan.name="tryFindPlan"
    plan.add_child(py_trees.behaviours.SuccessEveryN("dummySuccess",1))
    plan.blackbox_level = plan.blackbox_level.COMPONENT

    def remove():
        poses : PoseArray = py_trees.Blackboard().get(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE) 
        
        try:
            poses.poses.pop(0)
        except:
            return py_trees.Status.FAILURE
        py_trees.Blackboard().set(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,poses)
        return py_trees.Status.SUCCESS


    execute_plan = ExecuteMoveItPlan("eeToNextPoint",MoveGroup.ARM,success_on_no_plan=True)


    remove_pose = Lambda("popPose",remove)

    traj_sequence.add_children([publish_vis_s,check_not_completed,plan,execute_plan,remove_pose])
    traj_sequence = py_trees.decorators.Condition(traj_sequence,status=py_trees.Status.FAILURE)

    spray.add_children([move_target,traj_sequence])
    spray.blackbox_level = spray.blackbox_level.BIG_PICTURE
    return spray

def create_raise_ee_to_first_target_pose_z(target_pose_src="target_poses"):
    """ takes first target in moveit/target_poses and raises gripper to match height (plans in base_link) """


    sequence = py_trees.composites.Sequence()

    def process_pose():
        ArmCommander().set_goal_tolerance(MoveGroup.ARM,0.15)
        p = ArmCommander().get_current_pose(MoveGroup.ARM).pose
        target_poses : PoseArray = py_trees.Blackboard().get(target_pose_src)
        if target_poses:

            p.position.z = target_poses.poses[0].position.z
            pa = PoseArray()
            pa.poses = [p]
            pa.header = target_poses.header
            return pa
        else:
            return None

    move_target = SetBlackboardVariableCustom(
            variable_name=CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,
            variable_value=process_pose,
            name="moveTarget2BB")

    create_plan = CreateMoveitTrajectoryPlan("planRaiseEE",
        MoveGroup.ARM,0.9,
        pose_frame="base_link",
        pose_target_include_only_idxs=[0],
        )

    execute_plan = ExecuteMoveItPlan("raiseEE",MoveGroup.ARM)

    sequence.add_children([move_target,create_plan,execute_plan])

    return sequence

def create_move_in_front_of_door(handle_pose_src,spray_path_src,distance_from_door=0.1,name="moveToDoor"):
    """ positions the robot perpendicular to the door at specified distance from the normal in the direction of it 
    
        Args:
            handle_pose_src (`str->Pose`): 
            spray_path_src (`str->PoseArray`): 
    """

    def process_pose():
        pose : Pose = py_trees.Blackboard().get(handle_pose_src)
        if pose:
            handle_quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]

            
            (a,b,c) = t.euler_from_quaternion(handle_quat)
            mat = t.euler_matrix(a,b,c)
            vec =  (mat[:,:-1] @ np.array([[1],[0],[0]])* distance_from_door )

            
            pose.position.x += vec[0]
            pose.position.y += vec[1]
            pose.position.z += vec[2]
            
            rot = quat_from_yaw(math.pi)
            
            new_quat = t.quaternion_multiply(handle_quat,rot)
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = new_quat


            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = "base_link"
            ps.header.stamp = rospy.Time.now()

            g = MoveBaseGoal()
            g.target_pose = ps
            
            return  g
        else:
            return None

    bb2target = "{}/target".format(name)

    
    wait = py_trees.timers.Timer(duration=5)

    move_target = SetBlackboardVariableCustom(
        variable_name=bb2target,
        variable_value=process_pose,
        name="moveTarget2BB")

    disable_costmap_1 = DynamicReconfigure("disableGlobalCostmap","move_base/global_costmap/inflation_layer",{
        "enabled":False
    })

    disable_costmap_2 = DynamicReconfigure("disableLocalCostmap","move_base/local_costmap/inflation_layer",{
        "enabled":False
    })

    clear_costmaps = CallService("clearCostmaps","move_base/clear_costmaps",
                                Empty,
                                EmptyRequest(),
                                blackboard_target="//trash")

    move_base = create_move_base_to_reach_pose(target_pose_src=bb2target)

    # manually come a bit closer as well
    speed= 0.05
    distance = distance_from_door * 0.5
    time = distance / speed

    moveMsg = Twist()
    moveMsg.linear.x = speed
    moveMsg.angular.z = 0 


    move = PublishTopic(
        name="haltCmdVel",
        msg=moveMsg,
        msg_type=Twist,
        topic="/cmd_vel",
        success_after_n_publishes=2,
        success_on_publish=True
    )
    hold_move = HoldStateForDuration(move,"HoldFailureIgnoreRunning",time,state=py_trees.common.Status.SUCCESS,ignore_running=True)

    haltMsg = Twist()

    halt = PublishTopic(
        name="haltCmdVel",
        msg=haltMsg,
        msg_type=Twist,
        topic="/cmd_vel",
        success_after_n_publishes=2,
        success_on_publish=True
    ) 
    door_open =py_trees.composites.Sequence(name=name,children=[wait,move_target,
            disable_costmap_1,disable_costmap_2,clear_costmaps,move_base,hold_move,halt])
    
    door_open.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    door_open.name = "move2Door"

    return door_open


def create_localize_robot(name="localizeRobot"):
    """ generates random walk for 10 seconds and initializes from unknown location using amcl, requires the navigation stack to be running"""

    bb2target = "{}/".format(name)

    sequence = py_trees.composites.Sequence(name)

    # init_amcl = CallService("initGlobalLocalization",
    #                 service_topic="/global_localization",
    #                 service_type=Empty,
    #                 service_content=EmptyRequest(),
    #                 blackboard_target="//trash")


    # walk_sequence = py_trees.composites.Sequence()


    # def process_response(x : GenerateTargetResponse):
    #     if x.success:
    #         return x.goal
    #     else:
    #         raise Exception("Target was not generated") 

    # get_random_pos = CallService("getRandomTarget",
    #     service_topic="/target_generator/generate_nav_target",
    #     service_type=GenerateTarget,
    #     service_content=GenerateTargetRequest(),
    #     blackboard_target=bb2target,
    #     pre_process_callable=process_response)

    # def get_pose():
    #     return py_trees.Blackboard().get(bb2target)

    # reach_pose = ActionClientConnectOnInit(name,
    #                         MoveBaseAction,
    #                         get_pose,
    #                         action_namespace="/move_base",
    #                         override_feedback_message_on_running="moving to reach target")

    # walk_sequence.add_children([get_random_pos,reach_pose])

    # random_walk = py_trees.decorators.Timeout(walk_sequence,duration=30)

    # sequence.add_children([init_amcl,random_walk])
    sequence.add_children([py_trees.behaviours.SuccessEveryN("dummy",1)])
    return sequence

import sys
import inspect 
import types
import pydot
from graphviz import render

def is_function_local(object):
    return isinstance(object, types.FunctionType) and object.__module__ == __name__


# for generating dotfiles for each subtree
if __name__ == "__main__":

    trees = [
        ("create_explore_frontier_and_save_map",create_explore_frontier_and_save_map()),
        ("create_face_closest_obstacle",create_face_closest_obstacle()),
        ("create_disinfect_doors_in_map",create_disinfect_doors_in_map("","",""))

    ]

    

    dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..","..","docs","trees")

    for (n,t) in trees:
        t = py_trees.composites.Sequence(name=n,children=[t])
        py_graph = py_trees.display.generate_pydot_graph(t,py_trees.common.VisibilityLevel.COMPONENT)
        py_graph.write(os.path.join(dir,n+".dot"))
        py_graph.write_png(os.path.join(dir,n + ".png"))
    
       