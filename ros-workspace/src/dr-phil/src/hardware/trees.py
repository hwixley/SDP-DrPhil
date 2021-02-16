from time import time
import py_trees
import py_trees_ros
from behaviours import ClosestObstacle,PublishTopic,RunRos,KillSubprocess
import operator
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
import os


def create_explore_frontier_and_save_map(map_path=None,timeout=120):
    """ creates subtree which executes frontier exploration, generates a map and saves it to the given map_name """
    sequence = py_trees.composites.Sequence()

    startExplorationNodes = create_run_ros_once(
        launch_file="explore_task.launch",
        package="dr-phil",
        subprocess_variable_name="subprocess/explore")
    
    sequence2 = py_trees.composites.Sequence()

    # frontierEmptyCheck = py_trees_ros.subscribers.CheckData(name="frontierEmpty?",
    #                                     topic_name="explore/frontiers",
    #                                     topic_type=MarkerArray,
    #                                     variable_name="markers",
    #                                     # comparison_operator=operator.__eq__,
    #                                     # expected_value=[],
    #                                     fail_if_bad_comparison=True,
    #                                     fail_if_no_data=False)
    sequence2 = py_trees.composites.Sequence()


    idle = create_idle()
    timeout = py_trees.decorators.Timeout(idle,duration=timeout)
    inverter = py_trees.decorators.Inverter(timeout)
    oneShotTimeout = py_trees.decorators.OneShot(inverter)
    
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
    )
    timeoutHalt = py_trees.decorators.Timeout(halt,duration=1)

    sequence2.add_children([oneShotTimeout,saveMap,killExplorationNodes,timeoutHalt])

    sequence.add_children([startExplorationNodes,sequence2])

    oneShot = py_trees.decorators.OneShot(sequence)
    return oneShot

def create_run_ros_once(package,subprocess_variable_name,launch_file=None,node_file=None,watch_time=5,time_out=10,args=[]):
    """ creates subtree which attempts to launch a launchfile/node just once, after successful launch never runs again. 
    After each failure will attempt to retry untill time out is reached """
    launchBeh = RunRos(name="runLaunchfileOnce",
                                launch_file=launch_file,
                                node_file=node_file,
                                package=package,
                                subprocess_variable_name=subprocess_variable_name,
                                watch_time=watch_time,
                                args=args)
    
    oneShot = py_trees.decorators.OneShot(launchBeh)
    timeOut = py_trees.decorators.Timeout(oneShot,duration=time_out)
    return oneShot

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