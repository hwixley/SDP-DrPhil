#!/usr/bin/env python3
from socket import timeout
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker,MarkerArray

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

import dr_phil_hardware.vision.explore as explore


from sklearn.cluster import AgglomerativeClustering
#TODO:rospy.service.ServiceException: service [/move_base/make_plan] responded with an error: b''
#Happens when recovery behaviour or moving-  [1616849172.299558158, 905.107000000]: move_base must be in an inactive state to make a plan for an external user

#TODO: Segmentation fault: Order of opening nodes matters (happens when map_explorer is called before target_generator)
#TODO: Stopping motors - cant shutdown properly

#TODO: NoneType' object has no attribute 'target_pose' - repeat asking target




class MapExplorer:

    #defines the maximum distance of what defines an explored region (square shaped)
    REGION_EXPLORED_THRESHOLD = 1.5
    STOPPING_THRESHOLD = 0.70

    DOOR_WIDTH_THRESH= 1.3

    def __init__(self):
        self.global_costmap = None
        self.yaw = None
        self.robot_pose = Pose() # robot pose in map frame
        #TODO: might need to turn this to posearray
        self.spotted_door = None

        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)
        self.amcl_pose_sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, callback=self.robot_pose_callback)
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, callback=self.robot_orientation_callback)

        #Get door poses
        #TODO: might need to turn this to posearray


        #Defines the set of poses generated by random targets that we have been close to
        self.explored_regions = []
        #Defines the set of doors found in a map
        self.door_locations = []

        self.area_explored= 0
        self.total_area = 0
        self.map_resolution = 0
        self.get_map_area_and_resolution()

        self.number_of_movements = 0

        self.pub_visualization_marker = rospy.Publisher('/explored_regions', MarkerArray, queue_size=100)
        self.pub_exploration_visualization_marker = rospy.Publisher('/marked_cells', MarkerArray, queue_size=10)

        self.pub_door_visualization_marker = rospy.Publisher('/door/markers', MarkerArray, queue_size=100)



    def robot_pose_callback(self, poseWithCov : PoseWithCovarianceStamped):
        if poseWithCov:
            self.robot_pose = poseWithCov.pose.pose
    
    def door_pose_callback(self, door : PoseStamped):
        if door is not None:
            if self.spotted_door is None or self.spotted_door.pose.position != door.pose.position :
                self.spotted_door = door
                self.door_locations.append(door)

    def robot_orientation_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        #print(yaw)

    def global_costmap_callback(self, global_costmap : OccupancyGrid):
        self.global_costmap = global_costmap

    def spin(self):
        #self.custom_spin()
        if (self.area_explored/self.total_area)>=self.STOPPING_THRESHOLD:
            print("we explored everything!")
            rospy.signal_shutdown("Finished exploration")
       
        else:
            #Start with generating targets 
            distance_threshold = 2
            goal = explore.generate_random_target(distance_threshold)
            counter = 0
            while (goal is None  or not explore.is_location_available(self.robot_pose, goal) or self.was_explored(goal)):
                goal = explore.generate_random_target(distance_threshold)
                counter+=1
                #Every 5 iterations, increase the distance threshold. This is done in order to prevent the robot from getting stuck in case it cant find a suitable target.
                if (counter%10==0):
                    rospy.loginfo("Increasing the distance threshold")
                    distance_threshold+=0.5
            rospy.loginfo("Recieved random target")
            #print(mapExplorer.robot_pose)
            #print(explore.is_location_available(mapExplorer.robot_pose, goal))
            result = explore.move_to_goal(goal)
            if result:
                self.area_explored+=((1/self.map_resolution)*self.REGION_EXPLORED_THRESHOLD)*((1/self.map_resolution)*self.REGION_EXPLORED_THRESHOLD)
                self.explored_regions.append((goal))
                rospy.loginfo("Goal execution done!")
                # self.find_doors_nearby()
        self.mark_all_explored_regions()

    #If we've been near one of the random points generated, return true
    def was_explored(self, goal):
        for pixel in self.explored_regions:
            # pixels = (1/self.map_resolution)*self.REGION_EXPLORED_THRESHOLD
            # print(pixels)
            explored_x = pixel.target_pose.pose.position.x 
            goal_x = goal.target_pose.pose.position.x

            explored_y = pixel.target_pose.pose.position.y
            goal_y = goal.target_pose.pose.position.y



            if abs(explored_x - goal_x) <  self.REGION_EXPLORED_THRESHOLD and abs(explored_y - goal_y) < self.REGION_EXPLORED_THRESHOLD:
                return True
        return False

    def get_map_area_and_resolution(self):
        rospy.loginfo("Waiting for static_map")
        rospy.wait_for_service("static_map")
        try:
            map_service = rospy.ServiceProxy('static_map', GetMap)
            rospy.loginfo("Map Info retrieved")
            response = map_service()
            map_size_x = response.map.info.width
            map_size_y = response.map.info.height
            self.map_resolution = response.map.info.resolution
            self.total_area = len([x for x in response.map.data if x!=-1])
            print((self.total_area))
            print(self.map_resolution)


        except rospy.ServiceException as e:               
            rospy.logerr("Could not retrieve map info: %s"%e)
    
    def custom_spin(self):
        # goal = explore.get_test_goal(self.robot_pose)
        # result = explore.move_to_goal(goal)
        # if result:
            # rospy.loginfo("Goal execution done!")
        self.find_doors_nearby()
        # self.mark_all_doors()
        if self.door_locations is not None:
            #print(self.door_locations)
            #Only taking into account x,y points to indicate different doors
            X = [ np.array([door.pose.position.x,
                            door.pose.position.y]) for door in self.door_locations]

            print(X)
            if (len(self.door_locations) > 1):

                y_pred = AgglomerativeClustering(linkage='ward',distance_threshold=self.DOOR_WIDTH_THRESH, n_clusters=None).fit_predict(X)
                
                print(y_pred)
                ind = np.argsort(y_pred)
                
                self.door_locations = list(np.array(self.door_locations)[ind])
                self.mark_all_doors()



        
    def mark_all_explored_regions(self):
        markers = []
        id = 0
        for pixel in self.explored_regions:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = pixel.target_pose.header.stamp
            marker.ns = "turtlepi_navigate"
            marker.id = id
            marker.type =2 #Sphere
            marker.action = 0 #add/modify
            marker.pose.position.x = pixel.target_pose.pose.position.x
            marker.pose.position.y = pixel.target_pose.pose.position.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = pixel.target_pose.pose.orientation.x
            marker.pose.orientation.y =  pixel.target_pose.pose.orientation.y
            marker.pose.orientation.z =  pixel.target_pose.pose.orientation.z
            marker.pose.orientation.w =  pixel.target_pose.pose.orientation.w
            marker.scale.x =  self.REGION_EXPLORED_THRESHOLD
            marker.scale.y = self.REGION_EXPLORED_THRESHOLD
            marker.scale.z = 0.1
            marker.color.a = 0.6
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0)

            
            id+=1
            markers.append(marker)
            


        self.pub_visualization_marker.publish(markers)

    
    def mark_all_doors(self):
        markers = []
        id = 0
        for door in self.door_locations:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp =  rospy.Time.now()
            marker.ns = "door markers"
            marker.id = id
            marker.type =0 #Sphere
            marker.action = 0 #add/modify
            marker.pose.position.x = door.pose.position.x
            marker.pose.position.y = door.pose.position.y
            marker.pose.position.z = door.pose.position.z
            marker.pose.orientation.x = door.pose.orientation.x
            marker.pose.orientation.y = door.pose.orientation.y
            marker.pose.orientation.z = door.pose.orientation.z
            marker.pose.orientation.w = door.pose.orientation.w
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.6
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(50)

            
            id+=1
            markers.append(marker)
            


        self.pub_door_visualization_marker.publish(markers)




     
    #Once we reach a target point, rotate in place for 360 degrees (full circle) and look for doors
    def find_doors_nearby(self):
        # wait for the initial orientation so we don't start with None
        rospy.wait_for_message('/odom', Odometry)
  


        # original yaw angle
        startingYaw = self.yaw

        # Rotate for 5 degrees to begin with
        rospy.loginfo("Rotating...")
        while (explore.angles_close(startingYaw, self.yaw, tolerance=5) \
               and not rospy.is_shutdown()): # makes sure script is killed on ctrl-c
            explore.initiate_rotation()

        
        

        # Continue rotating until robot gets within 3 deg of the original orientation 
        while (not explore.angles_close(startingYaw, self.yaw, tolerance=3) \
               and not rospy.is_shutdown()):
            self.door_poses_sub = rospy.Subscriber('/door/pose', PoseStamped, callback=self.door_pose_callback)

        rospy.loginfo("Rotation stopped!")

        #print("Doors {}".format((self.door_locations)))

        #Run k-means
        #TODO: Get a prediction on number of clusters  in one rotation in case more than one door


        rospy.loginfo("Scan Finished")
        #print("startingYaw: {}\ncurrentYaw: {}\n".format(np.rad2deg(startingYaw), np.rad2deg(self.yaw)))
        explore.stop_in_place()
    
    def spot_doors(self):
        pass


if __name__ == "__main__":
    rospy.init_node("map_explorer", anonymous=False)
    rospy.loginfo("Running map_explorer node")
    rospy.on_shutdown(explore.stop_motors) # make sure the robot stops on ctrl-c
    mapExplorer = MapExplorer()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mapExplorer.spin()
        rate.sleep()