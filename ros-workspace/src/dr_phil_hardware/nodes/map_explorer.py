#!/usr/bin/env python3
from socket import timeout
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import dr_phil_hardware.vision.explore as explore
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped


class MapExplorer:
    def __init__(self):
        self.global_costmap = None
        self.robot_pose = Pose() # robot pose in map frame
        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)
        self.amcl_pose_sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, callback=self.robot_pose_callback)

    def robot_pose_callback(self, poseWithCov : PoseWithCovarianceStamped):
        if poseWithCov:
            self.robot_pose = poseWithCov.pose.pose

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

if __name__ == "__main__":
    rospy.init_node("map_explorer", anonymous=False)
    mapExplorer = MapExplorer()
    

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mapExplorer.spin()
        rate.sleep()
    