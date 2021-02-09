import py_trees
import py_trees_ros


def create_idle(): 
    """ creates a subtree which causes the robot to idle on each tick, with status of RUNNING """
    
    idle = py_trees.behaviours.Running(name="Idle")

    return idle