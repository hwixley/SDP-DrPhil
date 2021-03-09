
#!/usr/bin/env python3


from time import time

from numpy.core.fromnumeric import var
from dr_phil_hardware.arm_interface.command_arm import MoveGroup
import py_trees
from py_trees.composites import Parallel
from py_trees.decorators import FailureIsRunning
from py_trees.meta import oneshot
import py_trees_ros

from dr_phil_hardware.behaviour.leafs.ros import PublishTopic,RunRos,MessageChanged,JointTargetSetAndForget
from dr_phil_hardware.behaviour.leafs.general import ClosestObstacle

from dr_phil_hardware.behaviour.decorators import HoldStateForDuration

import operator
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import JointTrajectoryPoint
import os
import numpy as np
from std_msgs.msg import Float64MultiArray
from actionlib_msgs.msg import GoalStatusArray

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

    
    sequence = py_trees.composites.Sequence()
    
    position_arm = JointTargetSetAndForget(name,MoveGroup.ARM,parameters[1:-1])
    position_gripper = JointTargetSetAndForget(name,MoveGroup.gripper,[parameters[-1],-parameters[-1]])

    sequence.add_children([position_arm,position_gripper])

    return sequence

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

    saveMap = RunRos(name="runSaveMap",
        blackboard_alive_key="map/alive",
        blackboard_kill_trigger_key="map/kill",
        node_file="map_saver",
        package="map_server",
        args=["-f",map_path],
        success_on_non_error_exit=True) 

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


def create_idle(): 
    """ creates a subtree which causes the robot to idle on each tick, with status of RUNNING """
    
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
    
       