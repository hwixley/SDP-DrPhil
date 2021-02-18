from time import time
import py_trees
from py_trees.decorators import FailureIsRunning
from py_trees.meta import oneshot
import py_trees_ros
from behaviours import ClosestObstacle,PublishTopic,RunRos,KillSubprocess
from decorators import HoldStateForDuration
import operator
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import JointTrajectoryPoint
import os
import numpy as np
from std_msgs.msg import Float64MultiArray


def create_frontier_empty_for_duration_check(duration=60):
    """ creates subtree which returns SUCCESS if no data has been received on the `/explore/frontiers` topic for the given duration.
        Returns FAILURE if data comes through in this time. RUNNING is propagated whenever waiting for message

        Args:
            duration: the time after which if no data is received on `/explore/frontiers` to return SUCCESS
    """

    fail_no_data = py_trees_ros.subscribers.WaitForData(name="FrontierDataArrived?",
                                        topic_name="explore/frontiers",
                                        topic_type=MarkerArray,
                                        clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS)

    return HoldStateForDuration(fail_no_data,"HoldRunning",duration,state=py_trees.common.Status.RUNNING)


    

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

    resetArmPosition = py_trees.decorators.OneShot(
        create_set_positions_arm([0,0,-1.57,1.57,-1.57,-1],name="clearArmPosition"))

    startExplorationNodes = create_run_ros_once(
        launch_file="explore_task.launch",
        package="dr-phil",
        subprocess_variable_name="subprocess/explore")
    
    sequence2 = py_trees.composites.Sequence()
    oneshot_sequence2 = py_trees.decorators.OneShot(sequence2)

    frontierEmptyCheck = create_frontier_empty_for_duration_check(duration=no_data_timeout)

    map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"map")if map_path is None else map_path  

    saveMap = create_run_ros_once(node_file="map_saver",
                                    package="map_server",
                                    subprocess_variable_name="subprocess/map_saver",
                                    args=["-f",map_path])
    killExplorationNodes = KillSubprocess(name="killExplorationNodes",
        subprocess_variable_name="subprocess/explore")

    haltMsg = Twist()
    haltMsg.linear.x = 0
    haltMsg.angular.z = 0 

    halt = PublishTopic(
        name="haltCmdVel",
        msg=haltMsg,
        msg_type=Twist,
        topic="/cmd_vel",
        success_on_publish=True
    )

    sequence2.add_children([frontierEmptyCheck,saveMap,killExplorationNodes,halt])

    sequence.add_children([resetArmPosition,startExplorationNodes,oneshot_sequence2])

    timeout = py_trees.decorators.Timeout(sequence,duration=timeout)

    return timeout

@oneshot
def create_run_ros_once(package,subprocess_variable_name,launch_file=None,node_file=None,watch_time=5,time_out=10,args=[]):
    """ creates subtree which attempts to launch a launchfile/node just once, after successful launch never runs again. """

    runName = launch_file if launch_file is not None else node_file

    launchBeh = RunRos(name="Run {0}".format(runName,args),
                                launch_file=launch_file,
                                node_file=node_file,
                                package=package,
                                subprocess_variable_name=subprocess_variable_name,
                                watch_time=watch_time,
                                args=args)
    return launchBeh

def create_idle(): 
    """ creates a subtree which causes the robot to idle on each tick, with status of RUNNING """
    
    idle = py_trees.behaviours.Running(name="Idle")

    return idle




def create_face_closest_obstacle(min_distance = 0.5,face_angle=0): 
    """ returns behaviour sub-tree which instructs the robot to turn towards and travel to the nearest obstacle within distance of min_distance """    

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