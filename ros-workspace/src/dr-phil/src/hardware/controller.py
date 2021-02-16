#!/usr/bin/env python3
import rospy
import py_trees
import py_trees_ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from trees import create_idle,create_face_closest_obstacle,create_explore_frontier_and_save_map
import functools 
from visualization_msgs.msg import MarkerArray
import os

class Controller:
    """ the controller responsible for dictating behaviours to dr-phil. Every node is to be controlled via this script and no node should command behaviours without going through the controller """

    def __init__(self):
        
        self.root = self.create_tree() 
        # issue setup cycle on the tree with 10 ms timeout
        # not sure what the timeout does TODO: make sure this parameter is okay
        self.root.setup(10)

        # visualisation
        visitor = py_trees.visitors.SnapshotVisitor()
        self.root.add_post_tick_handler(
            functools.partial(self.post_tick_handler,visitor))
        self.root.visitors.append(visitor)


    def post_tick_handler(self,snapshot_visitor,behaviour_tree):
        print(py_trees.display.ascii_tree(behaviour_tree.root,
            snapshot_information=snapshot_visitor))

    def update(self):

        # tick the tree
        self.root.tick()

    def create_tree(self):
        """ creates the behaviour tree  for dr-phil """

        root = py_trees.composites.Parallel("drphil")

        # a branch for listening to data and putting it on the blackboard

        # we treat blackboard as one source of truth, keep everything synchronous and 
        # deterministic. Any topics used by other nodes are to be 
        # placed on the blackboard first before being used further down the tree
        # this means we do not have to deal with asynchronous behaviour locally

        topics2bb = py_trees.composites.Parallel("topics2bb")
        camera2bb = py_trees_ros.subscribers.ToBlackboard(name="camera2bb",
                                                            topic_name="/image",
                                                            topic_type=Image,
                                                            blackboard_variables={'image':None})
        scan2bb =  py_trees_ros.subscribers.ToBlackboard(name="scan2bb",
                                                            topic_name="/scan",
                                                            topic_type=LaserScan,
                                                            blackboard_variables={'scan':None})
        

        # priorities  branch for main tasks, the rest of the tree is to go here
        priorities = py_trees.composites.Selector("priorities")
        runMapper = create_explore_frontier_and_save_map()

        # for convenience we keep granular behaviours in their own python files
        # this will promote the re-use of behaviours
        idle = create_idle()

        root.add_child(topics2bb)
        topics2bb.add_child(camera2bb)
        topics2bb.add_child(scan2bb)

        root.add_child(priorities)
        priorities.add_children([runMapper])
        
        return py_trees_ros.trees.BehaviourTree(root=root)

    



if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)
    controller = Controller()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()