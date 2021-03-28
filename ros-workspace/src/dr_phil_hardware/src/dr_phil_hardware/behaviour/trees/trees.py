#!/usr/bin/env python3

from dr_phil_hardware.vision.utils import quat_from_yaw
from py_trees.blackboard import Blackboard, SetBlackboardVariable
from dr_phil_hardware.arm_interface.command_arm import  MoveGroup
import py_trees
from dr_phil_hardware.behaviour.leafs.ros import CallService, CreateMoveItMove, DynamicReconfigure, PublishTopic,RunRos,MessageChanged,ActionClientConnectOnInit,CreateMoveitTrajectoryPlan,ExecuteMoveItPlan
from dr_phil_hardware.behaviour.leafs.general import CheckIsOnSchedule, ClosestObstacle,Lambda,SetBlackboardVariableCustom, ShouldPreemptAndGoBase, WaitForNextClean
from dr_phil_hardware.behaviour.decorators import HoldStateForDuration
import operator
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped,PoseWithCovarianceStamped
import os
from std_msgs.msg import Float64MultiArray,Float64
from py_trees.common import BlackBoxLevel, Status
import rospy 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_srvs.srv import Empty,EmptyRequest
from dr_phil_hardware.utils import rotate_pose_by_yaw, quat_to_vec
import numpy as np
import math 
import copy
from dr_phil_hardware.srv import SetRobotStatus,SetRobotStatusRequest
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from enum import IntEnum

ROBOT_STATE_TARGET = "robot/state"
APP_RESET_RETURN_TOPIC = "app/reset_return"
SPRAY_COMMAND_TOPIC = "/spray_controller/command"
ARM_TRAVELING_STATE = [0,0,-1.57,1.57,-1.57,1.57]


class DrPhilStatus(IntEnum):
    IDLE_CHARGING = 0,
    CLEANING = 1,
    RETURNING_TO_BASE = 2,
    STUCK = 3,
    EXECUTING_HALT_PROCEDURE = 4,




def create_update_app_state(battery_src,name="updateAppState"):
    """Updates robot state from the given sources, communicates with app

    Args:
        battery_src ([type]): [description]
        robot_state_src ([type]): [description]
        name (str, optional): [description]. Defaults to "updateAppState".
    """
    APP_STATUS_SET_TOPIC = "/app/set_status"

    def get_data():
        battery : BatteryState = Blackboard().get(battery_src)
        robot_state : int = Blackboard().get(ROBOT_STATE_TARGET)

        req = SetRobotStatusRequest()
        
        if battery is not None:
            req.battery_level = battery.voltage
        if robot_state is not None:
            req.status = robot_state
        

        return req 

    return CallService("callAppStateUpdate",APP_STATUS_SET_TOPIC,SetRobotStatus,get_data,"//trash")


def create_set_robot_status(new_status : DrPhilStatus):
    """ Set the blackboard variable corresponding to robot status

    Args:
        new_status (DrPhilStatus): [description]
    """
    return SetBlackboardVariable(name="setStatus[{}]".format(new_status),variable_name=ROBOT_STATE_TARGET,variable_value=new_status.value)

def create_return_base(pose_src,name="returnHome"):
    """Creates behaviour which returns the robot to base if it's not already there,
    succeeds if this is done successfully, fails if cannot reach home position

    Args:
        pose_src ([type]): [description]
        name (str, optional): [description]. Defaults to "returnHome".

    Returns:
        [type]: [description]
    """
    HOME = np.array([0,0])
    TARGET_POSE_SRC = "home/pose"
    def check_home():
        pose : PoseWithCovarianceStamped = Blackboard().get(pose_src)
        
        if pose is None:
            return Status.SUCCESS

        pose_np = np.array([pose.pose.pose.position.x,
                            pose.pose.pose.position.y])
        
        if np.linalg.norm(pose_np - HOME) < 0.1:
            return Status.FAILURE
        else:
            return Status.SUCCESS


    home_sequence = py_trees.composites.Sequence()
    check_is_home = Lambda("check_home",check_home)
    reset_return_request = CallService("resetAppReturn",APP_RESET_RETURN_TOPIC,Empty,EmptyRequest(),blackboard_target="//trash")
    home_sequence.add_children([check_is_home,reset_return_request])

    set_return_home = create_set_robot_status(DrPhilStatus.RETURNING_TO_BASE)

    
    home = PoseStamped()
    home.header.stamp = rospy.Time.now()
    home.header.frame_id = "map"
    home.pose = Pose()
    home.pose.position.x = HOME[0]
    home.pose.position.y = HOME[1]
    home.pose.orientation.x,home.pose.orientation.y,home.pose.orientation.z,home.pose.orientation.w = quat_from_yaw(0)
    m = MoveBaseGoal()
    m.target_pose = home 

    sequence = py_trees.composites.Sequence()
    set_goal = SetBlackboardVariable("setHomeGoal",variable_name=TARGET_POSE_SRC,variable_value=home)
    move_home = create_move_base_to_reach_pose(TARGET_POSE_SRC,name="reachHome")
    # while this will not be reached if we didnt have to move to home
    # that situation will only occur after we reached home at least once, or started at home
    set_home = create_set_robot_status(DrPhilStatus.IDLE_CHARGING) 
    sequence.add_children([set_goal,move_home,set_home])

    return py_trees.composites.Selector(children=[home_sequence,set_return_home,sequence])

def create_preempt_return_base(schedule_src,pose_src,name="preemptCheck"):
    """Creates a check which succeeds if the robot needs to now preempt what it's doing and return to base, and fails otherwise

    Args:
        schedule_src ([type]): [description]
        pose_src ([type]): [description]
        name (str, optional): [description]. Defaults to "preemptCheck".

    Returns:
        [type]: [description]
    """
    sequence = py_trees.composites.Sequence()
    check = ShouldPreemptAndGoBase("checkPreemptRequired",schedule_src)
    return_home = create_return_base(pose_src)
    sequence.add_children([check,return_home])

    return sequence 
    


def create_check_on_according_to_schedule(schedule_src,name="isOnSchedule"):
    """creates subtree which returns failure if we are not on schedule or schedule does not exist,
    and succeeds if we are on schedule and should be cleaning

    Args:
        schedule_src ([type]): [description]
    """
    
    return CheckIsOnSchedule(name,schedule_src)

def create_wait_for_next_clean(schedule_src,name="waitForNextClean"):
    """creates subtree which returns failure if we are not on schedule or schedule does not exist,
    and succeeds if we are on schedule and should be cleaning

    Args:
        schedule_src ([type]): [description]
    """

    return WaitForNextClean(name,schedule_src)


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
        topic="/arm_body_position",
        success_after_n_publishes=2,
        queue_size=10
    )
    

    return position_arm

def create_set_gripper_position(value,name="setGripper"):
    low_lim = -1
    upper_lim = 1.57
    clean = min(upper_lim,max(low_lim,value))

    position = Float64()
    position.data = clean
    position_gripper = PublishTopic(
        name=name,
        msg=clean,
        msg_type=Float64,
        topic="/gripper_position",
        success_after_n_publishes=2,
        queue_size=10
    )
    
    return position_gripper

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



def create_open_door(handle_pose_src,map_path,pose_frame="map",name="openDoorSequence"):
    
    
    parallel = py_trees.composites.Parallel("disinfectDoors",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

    load_map = create_load_map(map_path=map_path + ".yaml")

    start_disinfection_nodes = RunRos(name="runDisinfectionNodes",
        blackboard_alive_key="/disinfect/alive",
        blackboard_kill_trigger_key="/kill",
        launch_file="disinfect_task.launch",
        package="dr_phil_hardware"
    )
    start_disinfection_nodes.blackbox_level = BlackBoxLevel.DETAIL

    
    open_door_sequence = py_trees.composites.Sequence(name)

    parallel.add_children([load_map,start_disinfection_nodes,open_door_sequence])

    bb2bufferPath = "/door_open/path"
    bb2bufferPose = "/door_open/pose"

    def snapshot_targets(snapshot=False):
            handle : PoseStamped = py_trees.Blackboard().get(handle_pose_src)

            if handle is not None:
                # save
                if snapshot:
                    import numbers
                    pa = PoseArray()
                    pa.header.frame_id = handle.header.frame_id
                    pa.header.stamp = handle.header.stamp
                    copyh = copy.deepcopy(handle.pose)
                    copyh = rotate_pose_by_yaw(math.pi - math.radians(0),copyh)

                    orientation = [copyh.orientation.x,copyh.orientation.y,copyh.orientation.z,copyh.orientation.w]
                    vec = 0.01 * quat_to_vec(orientation)
                   
                    copyh.position.x += float(vec[0])
                    copyh.position.y += float(vec[1])
                    pa.poses  = [copyh]


                    py_trees.Blackboard().set(bb2bufferPath,pa)
                    py_trees.Blackboard().set(bb2bufferPose,copy.deepcopy(handle))
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.RUNNING
    
    wait = py_trees.timers.Timer(name="wait",duration=2)

    # TEMPORARY \
    from move_base_msgs.msg import MoveBaseGoal
    m = MoveBaseGoal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = 0.87,0.87 - 1.5,0.24
    ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quat_from_yaw(-math.pi/2 * 3)
    
    m.target_pose = ps 

    set_goal=py_trees.blackboard.SetBlackboardVariable(variable_name="/temp/pose",variable_value=m)
    temp_move = create_move_base_to_reach_pose(
        target_pose_src="/temp/pose")


    temp_move_to_handle = py_trees.Sequence()
    temp_move_to_handle.add_children([set_goal,temp_move])
    # TEMPORARY /


    await_targets = Lambda("awaitTargets",snapshot_targets)
    
    wait_2 = py_trees.timers.Timer(name="wait",duration=2)

    # note we use the pose directly from the blackboard sent in not the buffer
    # since we didnt snapshot yet
    move_to_door = create_move_in_front_of_door(handle_pose_src,1)

    wait_3 = py_trees.timers.Timer(name="wait",duration=2)

    snapshot_target = Lambda("snapshotTargets",lambda:snapshot_targets(snapshot=True))


    reset_arm = create_set_positions_arm(ARM_TRAVELING_STATE)
    open_gripper = create_set_gripper_position(1.57)

    plan_to_handle = create_execute_spray_trajectory(bb2bufferPath,pose_frame,0,0.195,move_only=True)


    grip_sequence = py_trees.Sequence()

    close_gripper1 = create_set_gripper_position(1.2)
    close_gripper2 = create_set_gripper_position(0.4)
    close_gripper3 = create_set_gripper_position(0)
    close_gripper4 = create_set_gripper_position(-0.2)
    close_gripper5 = create_set_gripper_position(-0.4)
    close_gripper6 = create_set_gripper_position(-0.6)
    close_gripper7 = create_set_gripper_position(-0.78) 

    grip_sequence.add_children([close_gripper1,close_gripper2,close_gripper3,
    close_gripper4,close_gripper5,close_gripper6,close_gripper7 ])
    pull = Twist()
    pull.linear.x = -0.09 * 0.15 * 2.2
    pull.angular.z = 0.015 * 2 * 2.2

    initiate_pull = PublishTopic("initiatePull",pull,Twist,"/cmd_vel",queue_size=1,success_after_n_publishes=2)

    wait_4 = py_trees.timers.Timer(duration=25)

    stop_pull = PublishTopic("initiatePull",Twist(),Twist,"/cmd_vel",queue_size=1,success_after_n_publishes=2)

    
    bb2bufferPosenew = "/door_open/forward"
    def get_new_goal():
        pose : PoseStamped = Blackboard().get(bb2bufferPath)

        pose.header = rospy.Time.now()

        pose.pose = rotate_pose_by_yaw(math.pi,pose.pose)
        new_quat = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        vec = quat_to_vec(new_quat) * 5
        pose.pose.position.x += vec[0]
        pose.pose.position.y += vec[1]


        mg = MoveBaseGoal()
        mg.target_pose = pose

        Blackboard().set(bb2bufferPosenew,mg)

        return Status.SUCCESS


    open_grip = create_set_gripper_position(1.5)
    swing_arm_left = create_set_positions_arm([0,1.57,0,0,0,0])

    get_new_goal_ = Lambda("setNewGoal",lambda:get_new_goal)

    move_new_goal = create_move_base_to_reach_pose(bb2bufferPosenew)

    open_door_sequence.add_children([wait,temp_move_to_handle,await_targets,wait_2,move_to_door,
    wait_3,snapshot_target,reset_arm,open_gripper,plan_to_handle,grip_sequence,initiate_pull,
    wait_4,stop_pull,open_grip,swing_arm_left,get_new_goal_,move_new_goal])
    
    return parallel

def create_disinfect_doors_in_map(handle_pose_src,spray_path_src,pose_frame="map",map_path=None,distance_from_door=0.1,spray_time=0.1,name="disinfectDoors"):
    """[summary]

    Args:
        handle_pose_src (`str->PoseStamped`): 
        spray_path_src (`str->PoseArray`):
        map_path ([type], optional): [description]. path to file without extension.
        name (str, optional): [description]. Defaults to "disinfectDoors".
    """

    assert(not ".yaml" in map_path  or not ".pgm" in map_path)

    bb2bufferPose = "{}/handle_pose".format(name)
    bb2bufferSpray = "{}/spray_poses".format(name)
    killkey = "/disinfect/kill"
    # ------ start nodes ----------
    parallel = py_trees.composites.Parallel("disinfectDoors",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

    load_map = create_load_map(map_path=map_path + ".yaml")

    start_disinfection_nodes = RunRos(name="runDisinfectionNodes",
        blackboard_alive_key="/disinfect/alive",
        blackboard_kill_trigger_key=killkey,
        launch_file="disinfect_task.launch",
        package="dr_phil_hardware"
    )
    start_disinfection_nodes.blackbox_level = BlackBoxLevel.DETAIL

    # ------ do some movement ---------

    sequence = py_trees.composites.Sequence(name="mainSequence")
    sequence.blackbox_level = BlackBoxLevel.BIG_PICTURE


    # # TEMPORARY \
    # from move_base_msgs.msg import MoveBaseGoal
    # m = MoveBaseGoal()
    # ps = PoseStamped()
    # ps.header.frame_id = "map"
    # ps.header.stamp = rospy.Time.now()
    # ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = 0.87,0.87 - 1.5,0.24
    # ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quat_from_yaw(-math.pi/2 * 3)
    
    # m.target_pose = ps 

    # set_goal=py_trees.blackboard.SetBlackboardVariable(variable_name="/temp/pose",variable_value=m)
    # temp_move = create_move_base_to_reach_pose(
    #     target_pose_src="/temp/pose")


    # temp_move_to_handle = py_trees.Sequence()
    # temp_move_to_handle.add_children([set_goal,temp_move])
    # # TEMPORARY /

    set_disinfect = create_set_robot_status(DrPhilStatus.CLEANING)

    reset_arm = create_set_positions_arm(ARM_TRAVELING_STATE,name="resetArm")

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

    # note we use the pose directly from the blackboard sent in not the buffer
    # since we didnt snapshot yet
    move_to_door = create_move_in_front_of_door(handle_pose_src,1)

    wait_3 = py_trees.timers.Timer(name="wait",duration=2)

    snapshot_target = Lambda("snapshotTargets",lambda:snapshot_targets(snapshot=True))

    def get_spray():
        s :PoseArray= Blackboard().get(bb2bufferSpray)
        sc = copy.deepcopy(s)
        sc.poses = [sc.poses[0]]
        return sc

    publish_vis_s = PublishTopic("publishSprayTargets",get_spray,PoseArray,"/vis/spray_targets",success_on_publish=True)
    
    execute_spray_sequence = create_execute_spray_trajectory(bb2bufferSpray,pose_frame,
        spray_time,
        distance_from_pose=distance_from_door)

    kill_nodes = py_trees.blackboard.SetBlackboardVariable(name="killNodes",variable_name=killkey)
    
    sequence.add_children([set_disinfect,wait,reset_arm,localize,await_targets,wait_2,move_to_door,wait_3,snapshot_target,publish_vis_s,execute_spray_sequence,kill_nodes])

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
    """Reaches given move base goal 

    Args:
        target_pose_src ([type]): [description]
        name (str, optional): [description]. Defaults to "reachTargetPos".
    """


    def get_goal():
        return py_trees.Blackboard().get(target_pose_src)

    def get_pose():
        goal : MoveBaseGoal= get_goal()
        if goal is None:
            return PoseStamped()
        else:
            return goal.target_pose

    sequence = py_trees.composites.Sequence(name="reachTargetPosSequence")

    publish_vis_h = PublishTopic("publishMoveTarget",get_pose,PoseStamped,"/vis/handle_target",success_on_publish=True)

    move_base = ActionClientConnectOnInit(name,
                            MoveBaseAction,
                            get_goal,
                            action_namespace="/move_base",
                            override_feedback_message_on_running="moving to reach target")

    sequence.add_children([publish_vis_h,move_base])
    return sequence

def create_try_various_arm_plans(pose_frame):
    plan = py_trees.composites.Selector()

    # create_trajectory_plan = CreateMoveitTrajectoryPlan("planTrajectory",
    #     MoveGroup.ARM,0.9,
    #     pose_frame=pose_frame,
    #     pose_target_include_only_idxs=[0],
    #     )

    # create_lenient_trajectory_plan = CreateMoveitTrajectoryPlan("planTrajectoryLenient",
    #     MoveGroup.ARM,0.5,
    #     pose_frame=pose_frame,
    #     pose_target_include_only_idxs=[0],
    #     )

    # create_p2p_plan = CreateMoveItMove("planP2PMove",
    #     MoveGroup.ARM,
    #     pose_frame=pose_frame,
    #     goal_tolerance=0.01,
    #     )

    create_p2p_plan_lenient = CreateMoveItMove("planP2PMove",
        MoveGroup.ARM,
        pose_frame=pose_frame,
        goal_tolerance_orient=0.25,
        goal_tolerance_pos=0.03)

    plan.add_children([create_p2p_plan_lenient])
    return plan

def create_execute_spray_trajectory(target_pose_src,planning_frame,spray_time,distance_from_pose=0.1,move_only=False):

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

    check_not_completed = Lambda("checkNotCompleted",check)

    reset_arm = create_set_positions_arm(ARM_TRAVELING_STATE,name="resetArm")
    if move_only:
        reset_arm = py_trees.behaviours.Success()

    move_in_front = create_move_in_front_of_current_spray_pose(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE,distance_from_pose)
    move_in_front = py_trees.decorators.FailureIsSuccess(move_in_front)

    def get_spray():
        s = Blackboard().get(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE)
        sc = copy.deepcopy(s)
        sc.poses = [sc.poses[0]]
        return sc
    publish_vis_s = PublishTopic("publishSprayTargets",get_spray,PoseArray,"/vis/spray_targets",success_on_publish=True)

    
    plan = create_try_various_arm_plans(planning_frame)
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

    
    spray_duration = create_spray_for_duration(duration=spray_time)
    if move_only:
        spray_duration = py_trees.behaviours.Success()

    remove_pose = Lambda("popPose",remove)

    traj_sequence.add_children([check_not_completed,publish_vis_s,reset_arm,
                            move_in_front,plan,execute_plan,spray_duration,remove_pose])
    traj_sequence = py_trees.decorators.Condition(traj_sequence,status=py_trees.Status.FAILURE)

    spray.add_children([move_target,traj_sequence])
    spray.blackbox_level = spray.blackbox_level.BIG_PICTURE
    return spray

def create_spray_for_duration(duration=0.1,name="sprayDuration"):

    spray_sequence = py_trees.composites.Sequence(name=name)

    on_msg = Float32()
    on_msg.data = 1
    off_msg = Float32()
    off_msg.data = 0

    wait_before = py_trees.timers.Timer(duration=0.5,name="sprayBeforeWait")
    start_spray = PublishTopic("startSpray",on_msg,Float32,SPRAY_COMMAND_TOPIC,success_on_publish=True)
    wait = py_trees.timers.Timer(duration=duration,name="sprayInterval")
    stop_spray = PublishTopic("stopSpray",off_msg,Float32,SPRAY_COMMAND_TOPIC,success_on_publish=True)
    wait_after = py_trees.timers.Timer(duration=0.5,name="sprayAfterWait")
    spray_sequence.add_children([wait_before,start_spray,wait,stop_spray,wait_after])
    return spray_sequence

def create_move_in_front_of_current_spray_pose(spray_poses_src,distance_from_pose=0.1,name="moveToSprayPose"):
    """Moves in front of current pose at the given distance or the distance necessary to reach the pose with the end effector (further if target is closer to the height of hte base of the arm)

    Args:
        spray_poses_src ([type]): [description]
        distance_from_pose (float, optional): [description]. Defaults to 0.1.
        name (str, optional): [description]. Defaults to "moveToSprayPose".
    """
    def process_pose(distance_from_pose):
        poses : PoseArray = py_trees.Blackboard().get(spray_poses_src)
        
        if poses is None or len(poses.poses) == 0:
            return None

        pose : Pose = poses.poses[0]
        handle_quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]

        from random import random

        # simple fix for reachability of poses right at height of base of the arm
        distance_from_arm_base = abs(pose.position.z - 0.25)
        if distance_from_arm_base <= 0.07:
            distance_from_pose += 0.04
        
        vec = quat_to_vec(handle_quat) * -distance_from_pose

        new_pose = Pose()
        new_pose.position.x = vec[0] + pose.position.x
        new_pose.position.y = vec[1] + pose.position.y
        new_pose.position.z = vec[2] + pose.position.z
        new_pose.orientation = copy.deepcopy(pose.orientation)

        ps = PoseStamped()
        ps.pose = new_pose
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = poses.header.frame_id
        g = MoveBaseGoal()
        g.target_pose = ps
        
        return  g

    bb2target = "{}/target".format(name)

    move_target = SetBlackboardVariableCustom(
        variable_name=bb2target,
        variable_value=lambda : process_pose(distance_from_pose),
        name="moveTarget2BB")

    # disable_costmap_1 = DynamicReconfigure("disableGlobalCostmap","move_base/local_costmap/inflation_layer",{
    #     "inflation_radius":0.05
    # })

    disable_costmap_2 = DynamicReconfigure("disableLocalCostmap","/move_base/TebLocalPlannerROS",{
        "min_obstacle_dist":0.05,
        "inflation_dist":0.10
    })

    clear_costmaps = CallService("clearCostmaps","move_base/clear_costmaps",
                                Empty,
                                EmptyRequest(),
                                blackboard_target="//trash")

    move_base = create_move_base_to_reach_pose(target_pose_src=bb2target)

    # enable_costmap_1 = DynamicReconfigure("enableGlobalCostmap","move_base/local_costmap/inflation_layer",{
    #     "inflation_radius":0.22
    # })

    enable_costmap_2 = DynamicReconfigure("enableLocalCostmap","/move_base/TebLocalPlannerROS",{
        "min_obstacle_dist":0.22,
        "inflation_dist":0.44
    })
  
    door_open =py_trees.composites.Sequence(name=name,children=[move_target,
            disable_costmap_2,clear_costmaps,move_base,enable_costmap_2])
    
    door_open.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    door_open.name = "move2SprayPose"

    return door_open


def create_move_in_front_of_door(handle_pose_src,distance_from_door=0.1,name="moveToDoor"):
    """ positions the robot perpendicular to the door at specified distance from the normal in the direction of it 
    
        Args:
            handle_pose_src (`str->PoseStamped`): 
    """

    def process_pose():
        pose_original : PoseStamped = py_trees.Blackboard().get(handle_pose_src)
        if pose_original:
            
            handle_quat = [pose_original.pose.orientation.x,
                pose_original.pose.orientation.y,
                pose_original.pose.orientation.z,
                pose_original.pose.orientation.w]

            pose_clone = copy.deepcopy(pose_original)

            vec = quat_to_vec(handle_quat) * distance_from_door

            pose_clone.pose.position.x += vec[0]
            pose_clone.pose.position.y += vec[1]
            pose_clone.pose.position.z += vec[2]
            
            rotated_pose = rotate_pose_by_yaw(math.pi,pose_clone.pose)

            pose_clone.pose.orientation = rotated_pose.orientation
            g = MoveBaseGoal()
            g.target_pose = pose_clone
            
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


    door_open =py_trees.composites.Sequence(name=name,children=[wait,move_target,
            disable_costmap_1,disable_costmap_2,clear_costmaps,move_base])
    
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
    
       