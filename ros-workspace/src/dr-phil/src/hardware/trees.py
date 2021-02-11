import py_trees
import py_trees_ros
from behaviours import ClosestObstacle,PublishTopic
import operator
from geometry_msgs.msg import Twist

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