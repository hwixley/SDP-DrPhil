#!/usr/bin/env python3
from py_trees.visitors import SnapshotVisitor
import rospy
import py_trees
import py_trees_ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from trees import create_idle,create_face_closest_obstacle,create_explore_frontier_and_save_map
import functools 
from visualization_msgs.msg import MarkerArray
import os
from py_trees import console 
import json 
from rospy.exceptions import ROSException
class Controller:
    """ the controller responsible for dictating behaviours to dr-phil. Every node is to be controlled via this script and no node should command behaviours without going through the controller """

    def __init__(self):

        py_trees.logging.level = py_trees.logging.Level.INFO # set this to info for more information

        self.root = self.create_tree() 
        # issue setup cycle on the tree with 10 ms timeout
        # not sure what the timeout does TODO: make sure this parameter is okay
        self.root.setup(10)

        # visualisation
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        
        self.root.add_post_tick_handler(
            functools.partial(self.tree_display,snapshot_visitor))


        self.root.visitors.append(snapshot_visitor)


    def tree_display(self,snapshot_visitor,behaviour_tree):
        print(py_trees.display.ascii_tree(behaviour_tree.root,
            snapshot_information=snapshot_visitor))

    def logger(self,snapshot_visitor, behaviour_tree):
        """
        A post-tick handler that logs the tree (relevant parts thereof) to a yaml file.
        """
        tree_serialisation = {
            'tick': behaviour_tree.count,
            'nodes': []
        }
        for node in behaviour_tree.root.iterate():
            node_type_str = "Behaviour"
            for behaviour_type in [py_trees.composites.Sequence,
                                py_trees.composites.Selector,
                                py_trees.composites.Parallel,
                                py_trees.decorators.Decorator]:
                if isinstance(node, behaviour_type):
                    node_type_str = behaviour_type.__name__
            node_snapshot = {
                'name': node.name,
                'id': str(node.id),
                'parent_id': str(node.parent.id) if node.parent else "none",
                'child_ids': [str(child.id) for child in node.children],
                'tip_id': str(node.tip().id) if node.tip() else 'none',
                'class_name': str(node.__module__) + '.' + str(type(node).__name__),
                'type': node_type_str,
                'status': node.status.value,
                'message': node.feedback_message,
                'is_active': True if node.id in snapshot_visitor.running_nodes else False
                }
            tree_serialisation['nodes'].append(node_snapshot)
        if behaviour_tree.count == 0:
            with open('dump.json', 'w+') as outfile:
                json.dump(tree_serialisation, outfile, indent=4)
        else:
            with open('dump.json', 'a') as outfile:
                json.dump(tree_serialisation, outfile, indent=4)


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

        runMapper = create_explore_frontier_and_save_map(timeout=600,no_data_timeout=120)
        mapperOneShot = py_trees.decorators.OneShot(runMapper)

        # for convenience we keep granular behaviours in their own python files
        # this will promote the re-use of behaviours
        idle = create_idle()

        root.add_child(topics2bb)
        topics2bb.add_children([camera2bb,scan2bb])

        root.add_child(priorities)
        priorities.add_children([mapperOneShot])
        
        return py_trees_ros.trees.BehaviourTree(root=root)

    



if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)
    controller = Controller()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            controller.update()
            rate.sleep()
        except (KeyboardInterrupt,ROSException):
            rospy.logwarn("Calling stop() on root behaviour")
            controller.root.destroy()