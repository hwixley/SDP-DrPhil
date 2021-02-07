#!/usr/bin/env python3

import rospy
import py_trees
import py_trees_ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

class Controller:

    def __init__(self):
        self.root = self.create_tree()
        self.root.setup(10)

    def update(self):
        self.root.tick()

    def create_tree(self):
        """ creates the behaviour tree  for dr-phil """

        root = py_trees.composites.Parallel("drphil")

        # a branch for listening to data and putting it on the blackboard

        # we treat blackboard as one source of truth, keep everything synchronous and 
        # deterministic
        topics2bb = py_trees.composites.Sequence("topics2bb")
        camera2bb = py_trees_ros.subscribers.ToBlackboard( name="camera2bb",
                                                            topic_name="/image",
                                                            topic_type=Image,
                                                            blackboard_variables={'image':None})
        scan2bb =  py_trees_ros.subscribers.ToBlackboard(name="scan2bb",
                                                            topic_name="/scan",
                                                            topic_type=LaserScan,
                                                            blackboard_variables={'scan':None})
       

        # priorities  branch for main tasks

        priorities = py_trees.composites.Selector("priorities")
        idle = py_trees.behaviours.Running(name="Idle")

        root.add_child(topics2bb)
        topics2bb.add_child(camera2bb)
        topics2bb.add_child(scan2bb)

        root.add_child(priorities)
        priorities.add_child(idle)
        
        return py_trees_ros.trees.BehaviourTree(root=root)




if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)
    controller = Controller()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()