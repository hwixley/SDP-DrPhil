#!/usr/bin/env python3
from socket import timeout
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
import dr_phil_hardware.vision.explore as explore
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

class MapExplorer:

    def __init__(self):
        self.global_costmap = None
        self.yaw = None
        self.robot_pose = Pose() # robot pose in map frame
        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)
        self.amcl_pose_sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, callback=self.robot_pose_callback)
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, callback=self.robot_orientation_callback)

    def robot_pose_callback(self, poseWithCov : PoseWithCovarianceStamped):
        if poseWithCov:
            self.robot_pose = poseWithCov.pose.pose

    def robot_orientation_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        #print(yaw)

    def global_costmap_callback(self, global_costmap : OccupancyGrid):
        self.global_costmap = global_costmap

    def spin(self):
        goal = explore.generate_random_target()
        #print(mapExplorer.robot_pose)
        #print(explore.is_location_available(mapExplorer.robot_pose, goal))
    
        result = explore.move_to_goal(goal)
        if result:
            rospy.loginfo("Goal execution done!")
        # tf_goal = goal
        
        # self.listener = tf.TransformListener()
        # self.listener.waitForTransform(
        #     'map',
        #     'base_link',
        #     rospy.Time(0),
        #     timeout=rospy.Duration(2)
        # )

        # tf_goal.target_pose = self.listener.transformPose('map', goal.target_pose)
        # #print(self.robot_pose)
        
        # #print("goal: {}\n\n robot_pose: {}\n\n".format(goal, self.robot_pose))
        # print(explore.is_location_available(self.robot_pose, tf_goal))

    def custom_spin(self):
        # wait for the initial orientation so we don't start with None
        rospy.wait_for_message('/odom', Odometry)

        # original yaw angle
        startingYaw = self.yaw

        # Rotate for 15 degrees to begin with
        while (explore.angles_close(startingYaw, self.yaw, tolerance=15) \
               and not rospy.is_shutdown()): # makes sure script is killed on ctrl-c
            explore.initiate_rotation()
        
        # Continue rotating until robot gets within 5 deg of the original orientation 
        while (not explore.angles_close(startingYaw, self.yaw, tolerance=5) \
               and not rospy.is_shutdown()):
            explore.initiate_rotation()

        print("Completed rotation")
        #print("startingYaw: {}\ncurrentYaw: {}\n".format(np.rad2deg(startingYaw), np.rad2deg(self.yaw)))

if __name__ == "__main__":
    rospy.init_node("map_explorer", anonymous=False)
    rospy.on_shutdown(explore.stop_motors) # make sure the robot stops on ctrl-c
    mapExplorer = MapExplorer()
    mapExplorer.custom_spin()
    """
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mapExplorer.custom_spin()
        rate.sleep()
    """