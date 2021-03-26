#!/usr/bin/env python3
from socket import timeout
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import dr_phil_hardware.vision.explore as explore
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped


class HandleLocator:
    def __init__(self):
        #Initialise node
        rospy.init_node("mark_handles", anonymous=False)
        self.global_costmap = None
        self.door_pose = None
        # robot pose in map frame
        self.robot_pose = None

        self.explored_areas = []

        #Initialise subscribers and get relevant data
        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self.robot_pose_callback)
        self.door_pose = rospy.Subscriber('/door/pose', PoseStamped, callback=self.door_pose_callback)



    def global_costmap_callback(self, global_costmap : OccupancyGrid):
        self.global_costmap = global_costmap


    def robot_pose_callback(self, poseWithCov : PoseWithCovarianceStamped):
        if poseWithCov:
            self.robot_pose = poseWithCov.pose.pose

    def door_pose_callback(self, pose: PoseStamped):
        self.door_pose = pose


    

    def spin(self):
        goal = explore.generate_random_target()
        result = explore.move_to_goal(goal)
        if result:
            rospy.loginfo("Goal execution done!")
            explore.rotate360()
            print
            if self.door_pose is not None:
                find_doors_nearby()

        

    def find_doors_nearby(self):
            if self.door
        

     

if __name__ == "__main__":
    handleLocator = HandleLocator()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        handleLocator.spin()
        rate.sleep()
    