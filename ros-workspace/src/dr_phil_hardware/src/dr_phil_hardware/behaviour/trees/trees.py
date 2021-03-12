#!/usr/bin/env python3

from py_trees.blackboard import Blackboard
from dr_phil_hardware.arm_interface.command_arm import ArmCommander, MoveGroup
import py_trees
from dr_phil_hardware.behaviour.leafs.ros import PublishTopic,RunRos,MessageChanged,ActionClientConnectOnInit,CreateMoveitTrajectoryPlan,ExecuteMoveItPlan
from dr_phil_hardware.behaviour.leafs.general import ClosestObstacle, Lambda,SetBlackboardVariableCustom,CheckFileExists
from dr_phil_hardware.behaviour.decorators import HoldStateForDuration
import operator
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
import os
from std_msgs.msg import Float64MultiArray
import rospy 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import tf.transformations as t 
import numpy as np

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

    neutral_pos = [0,0,0,0,0,0] # [0,0,-1.57,1.57,-1.57,-1]
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

    timeout = py_trees.decorators.Timeout(sequence,duration=timeout)

    return timeout



    
def create_disinfect_doors_in_map(handle_pose_src,spray_path_src,map_path=None,distance_from_door=0.1,name="disinfectDoors"):
    """[summary]

    Args:
        handle_pose_src (`str->Pose`): 
        spray_path_src (`str->PoseArray`):
        map_path ([type], optional): [description]. path to file without extension.
        name (str, optional): [description]. Defaults to "disinfectDoors".
    """

    assert(not ".yaml" in map_path  or not ".pgm" in map_path)

    # ------ start nodes ----------
    parallel = py_trees.composites.Parallel()

    load_map = create_load_map(map_path=map_path + ".yaml")

    start_disinfection_nodes = RunRos(name="runDisinfectionNodes",
        blackboard_alive_key="/disinfect/alive",
        blackboard_kill_trigger_key="/disinfect/kill",
        launch_file="disinfect_task.launch",
        package="dr_phil_hardware"
    )

    # ------ do some movement ---------

    sequence = py_trees.composites.Sequence()

    wait_for_targets = Lambda("waitForHandlePose",
        lambda : py_trees.Status.SUCCESS if 
            py_trees.Blackboard().get(handle_pose_src) is not None 
                else py_trees.Status.RUNNING)

    wait_for_spray = Lambda("waitForSprayPoses",
        lambda : py_trees.Status.SUCCESS if 
            py_trees.Blackboard().get(handle_pose_src) is not None 
                else py_trees.Status.RUNNING)

    move_to_door = create_move_in_front_of_door(handle_pose_src,spray_path_src,distance_from_door=distance_from_door,name="moveToDoor")

    sequence.add_children([wait_for_targets,wait_for_spray,move_to_door])

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
            (a,b,c) = t.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
            mat = t.euler_matrix(a,b,c)
            rospy.logerr(mat)
            vec =  (mat[:,:-1] @ np.array([[1],[0],[0]])) * distance_from_door
            
            pose.position.x += vec[0]
            pose.position.y += vec[1]
            pose.position.z += vec[2]
            pose.orientation.w *= -1
            ps = PoseStamped()
            ps.pose = pose
            ps.header.frame_id = "base_link"
            ps.header.stamp = rospy.Time.now()

            g = MoveBaseGoal()
            g.target_pose = ps
            
            return  g
        else:
            return None

    wait = py_trees.timers.Timer(name="waitForNodes",duration=5)
    bb2target = "{}/target".format(name)
    move_target = SetBlackboardVariableCustom(
        variable_name=bb2target,
        variable_value=process_pose,
        name="moveTarget2BB")

    move_base = create_move_base_to_reach_pose(target_pose_src=bb2target)

    raise_ee = create_raise_ee_to_first_target_pose_z(spray_path_src)

    return py_trees.composites.Sequence(name=name,children=[wait,move_target,move_base,raise_ee])

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
        ("create_face_closest_obstacle",create_face_closest_obstacle())
    ]

    

    dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"docs","trees")

    for (n,t) in trees:
        t = py_trees.composites.Sequence(name=n,children=[t])
        py_graph = py_trees.display.generate_pydot_graph(t,py_trees.common.VisibilityLevel.ALL)
        py_graph.write(os.path.join(dir,n+".dot"))
        py_graph.write_png(os.path.join(dir,n + ".png"))
    
       