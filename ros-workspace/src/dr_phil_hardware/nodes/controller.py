#!/usr/bin/env python3
from py_trees import blackboard
import rospy
import py_trees
import py_trees_ros
from sensor_msgs.msg import Image,LaserScan,BatteryState
from dr_phil_hardware.behaviour.leafs.general import CheckFileExists
from dr_phil_hardware.behaviour.leafs.ros import TransformToBlackboard
from dr_phil_hardware.behaviour.trees.trees import DrPhilStatus, ROBOT_STATE_TARGET, create_explore_frontier_and_save_map,create_disinfect_doors_in_map, create_find_handles, create_localize_robot, create_check_on_according_to_schedule, create_preempt_return_base, create_update_app_state,create_wait_for_next_clean
import functools 
import os
import json 
from rospy.exceptions import ROSException
from geometry_msgs.msg import PoseArray,PoseStamped,PoseWithCovarianceStamped
import sys
from dr_phil_hardware.msg import CleaningSchedule
from visualization_msgs.msg import MarkerArray

class Controller:
    """ the controller responsible for dictating behaviours to dr-phil. Every node is to be controlled via this script and no node should command behaviours without going through the controller """

    SPRAY_PATH_SOURCE="spray_path/target_points"
    HANDLE_POSE_SOURCE="handle_feature/pose"
    SCHEDULE_SOURCE="app/rdata"
    MAP_TO_ROB_TRANSFORM_SOURCE="tf/map/robot"
    ROBOT_POSE_SOURCE="robot/pose"
    BATTERY_SOURCE="battery_state"
    FINAL_DOOR_SOURCE="/door/final_markers"
    FINAL_DOOR_POSES_SOURCE="/door/pose_array"

    def __init__(self,ignore_schedule=False):

        py_trees.logging.level = py_trees.logging.Level.INFO # set this to info for more information

        
        self.root = self.create_tree(ignore_schedule) 
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

    def create_tree(self,ignore_schedule):
        """ creates the behaviour tree  for dr-phil """
        # set initial robot state
        blackboard.Blackboard().set(ROBOT_STATE_TARGET,DrPhilStatus.IDLE_CHARGING.value)

        map_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"map")

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
        

        target2bb= py_trees_ros.subscribers.ToBlackboard(name="sprayPoses2bb",
                                                        topic_name=Controller.SPRAY_PATH_SOURCE,
                                                        topic_type=PoseArray,
                                                        blackboard_variables={Controller.SPRAY_PATH_SOURCE:None})

        handle2bb= py_trees_ros.subscribers.ToBlackboard(name="handle2bb",
                                                        topic_name=Controller.HANDLE_POSE_SOURCE,
                                                        topic_type=PoseStamped,
                                                        blackboard_variables={Controller.HANDLE_POSE_SOURCE:None})
        
        schedule2bb= py_trees_ros.subscribers.ToBlackboard(name="schedule2bb",
                                                        topic_name=Controller.SCHEDULE_SOURCE,
                                                        topic_type=CleaningSchedule,
                                                        blackboard_variables={Controller.SCHEDULE_SOURCE:None})
        
        pose2bb= py_trees_ros.subscribers.ToBlackboard(name="pose2bb",
                                                        topic_name=Controller.ROBOT_POSE_SOURCE,
                                                        topic_type=PoseWithCovarianceStamped,
                                                        blackboard_variables={Controller.ROBOT_POSE_SOURCE:None})

        battery2bb= py_trees_ros.subscribers.ToBlackboard(name="battery2bb",
                                                        topic_name=Controller.BATTERY_SOURCE,
                                                        topic_type=BatteryState,
                                                        blackboard_variables={Controller.BATTERY_SOURCE:None})
        
        doors2bb= py_trees_ros.subscribers.ToBlackboard(name="doors2bb",
                                                        topic_name=Controller.FINAL_DOOR_SOURCE,
                                                        topic_type=MarkerArray,
                                                        blackboard_variables={Controller.FINAL_DOOR_SOURCE:None})
        
        update_app_state = py_trees.composites.Sequence(children=[
            py_trees.timers.Timer(duration=20),
            create_update_app_state(Controller.BATTERY_SOURCE)
        ])

        if ignore_schedule:
            update_app_state = py_trees.behaviours.Running()

        topics2bb.add_children([camera2bb,scan2bb,target2bb,handle2bb,schedule2bb,pose2bb,battery2bb,update_app_state,doors2bb])


        # priorities  branch for main tasks, the rest of the tree is to go here

        priorities = py_trees.composites.Selector("priorities")

        pre_empt_check = create_preempt_return_base(Controller.SCHEDULE_SOURCE,Controller.HANDLE_POSE_SOURCE)        
        if ignore_schedule:
            pre_empt_check = py_trees.behaviours.Failure()

        non_preempt_tasks = py_trees.composites.Chooser("nonPreempt")


        disinfection_sequence = py_trees.composites.Sequence("disinfectionSequence")

        on_schedule = py_trees.decorators.FailureIsRunning(create_check_on_according_to_schedule(Controller.SCHEDULE_SOURCE))
        if ignore_schedule:
            on_schedule = py_trees.behaviours.Success()

        map_guard = py_trees.composites.Sequence()


        map_doesnt_exist =  py_trees.decorators.Inverter(CheckFileExists("mapExists",map_path + ".pgm"))

        run_mapper = py_trees.decorators.OneShot(
            create_explore_frontier_and_save_map(timeout=600,no_data_timeout=60,map_path=map_path))

        map_guard.add_children([map_doesnt_exist,run_mapper])


        handle_explore_guard = py_trees.composites.Sequence()
        map_exists =  CheckFileExists("mapExists",map_path + ".pgm")

        explore_handles = create_find_handles(map_path,Controller.FINAL_DOOR_SOURCE,Controller.FINAL_DOOR_POSES_SOURCE)

        handle_explore_guard.add_children([map_exists,explore_handles])

        disinfect_guard = py_trees.composites.Sequence()

        map_exists =  CheckFileExists("mapExists",map_path + ".pgm")

        disinfect_doors = create_disinfect_doors_in_map(handle_pose_src=Controller.HANDLE_POSE_SOURCE,
            spray_path_src=Controller.SPRAY_PATH_SOURCE,
            map_path=map_path,
            distance_from_door=0.24,
            spray_time=0.1)
            
        wait_for_next_clean = create_wait_for_next_clean(Controller.SCHEDULE_SOURCE)
        if ignore_schedule:
            wait_for_next_clean = py_trees.behaviours.Success()

        disinfect_guard.add_children([map_exists,disinfect_doors,wait_for_next_clean])


        disinfection_sequence.add_children([on_schedule,disinfect_guard])

        non_preempt_tasks.add_children([handle_explore_guard,disinfection_sequence])
   
        priorities.add_children([pre_empt_check,non_preempt_tasks])
        priorities = py_trees.decorators.SuccessIsRunning(priorities)

        root.add_children([topics2bb,priorities])

        
        return py_trees_ros.trees.BehaviourTree(root=root)

    



if __name__ == "__main__":
    rospy.init_node("controller",anonymous=True)


    ignore_schedule = "-i" in sys.argv

    print("ignoring schedule")

    controller = Controller(ignore_schedule=ignore_schedule)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            controller.update()
            rate.sleep()
        except (KeyboardInterrupt,ROSException) as E:
            rospy.logwarn("Calling stop() on root behaviour")

            controller.root.destroy()
            sys.exit(0)