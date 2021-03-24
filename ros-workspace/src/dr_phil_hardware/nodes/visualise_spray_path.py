#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import PointStamped,Point,PoseArray,Pose
import resource_retriever as Retriever
import rospy
import sys

#Library to calculate points around handle To move Arm to to spray the handle from all sides
from dr_phil_hardware.disinfection.spray_path import get_position_and_orientation_of_spray_points
from dr_phil_hardware.disinfection.spray_path import Coord

from dr_phil_hardware.vision.localisation import *
from dr_phil_hardware.vision.vision_handle_axis_algorithm import define_handle_features_heursitic
 


class SprayPathVisualiser:
    def __init__(self):

        rospy.init_node('SprayPath',anonymous=True)
        #Initialise variables
        self.handle3D = None
        self.normal = None
        self.spray_origin_poses = None 
        self.spray_endpoints_poses= None

        #TODO: Initialise this in a root folder or in utils. To keep consistency
        self.robot_frame = "base_link"


        #Initialise subscribers to recieve the handle in 3D and the normal to the door 
        self.handle3D_sub = rospy.Subscriber("/handle_feature/handle3D",Float64MultiArray,callback=self.handle3D_callback)
        self.normal_sub = rospy.Subscriber("/handle_feature/normal",Float64MultiArray,callback=self.normal_callback)
        

        #Initialise publishers for the spray path calculations
        self.spray_origin_pub = rospy.Publisher("/spray_path/target_points",PoseArray,queue_size=10)
        self.spray_endpoints_pub = rospy.Publisher("/spray_path/direction_points",PoseArray,queue_size=10)


        
        #Initialise publishers for marking
        self.door_pub = rospy.Publisher('/door_visualisation', Marker, queue_size=10)
        self.vis_pub = rospy.Publisher('/spray_path_visualisation', Marker, queue_size=100)
        self.vis_arrow_pub = rospy.Publisher('/spray_direction_visualisation', MarkerArray, queue_size=100)
       

    def handle3D_callback(self,point):
        self.handle3D = point.data
    
    def normal_callback(self,direction):
        self.normal = direction.data

    #Call the library finction get_position_and_orientation_of_spray_points, which will help us generate variables of type PoseArrays to publish
    def publish_spray_poses(self):
        self.spray_origin_poses, self.spray_endpoints_poses = get_position_and_orientation_of_spray_points(self.handle3D,self.normal,self.robot_frame)
        #Publish Pose Results
        self.spray_origin_pub.publish(self.spray_origin_poses)
        self.spray_endpoints_pub.publish(self.spray_endpoints_poses)




    def visualise(self):  
        #Call the library function to publish the spray path poses at the end
        self.publish_spray_poses()
   

        if self.spray_origin_poses and self.spray_endpoints_poses:
            pass
            #Visualise markers and publish them for rviz to view them
            # points = self.show_path_points(self.spray_origin_poses)
            # self.vis_pub.publish(points)

            # path_direction = self.visualise_spray_direction(self.spray_origin_poses,self.spray_endpoints_poses)
            # self.vis_arrow_pub.publish(path_direction)

            # door_marker = self.display_door_and_handle(2)
            # self.door_pub.publish(door_marker)



    


    #TODO: change ids 
    def show_path_points(self,origin_poses):
        points = []
        for i in range(0,len(origin_poses.poses)):
            points.append(origin_poses.poses[i].position)

        arm_target_points = Marker()
        arm_target_points.header.frame_id = self.robot_frame
        arm_target_points.header.stamp = rospy.Time.now()
        arm_target_points.ns = "spray positions"
        arm_target_points.id = 0
        arm_target_points.type = 8 # sphere list
        arm_target_points.action = 0 # add/modify
        arm_target_points.points = points
        arm_target_points.pose.orientation.w = 1
        arm_target_points.color.a = 1
        arm_target_points.color.b = 1
        arm_target_points.scale.x = 0.01
        arm_target_points.scale.y = 0.01
        arm_target_points.scale.z = 0.01
        arm_target_points.lifetime = rospy.Duration(10)
        return arm_target_points
        
    #Visualise position of the door and handle in rviz as a marker
    #Note that the pose positions are hardcoded corresponding to the door in the map coordinates of the *house* world
    def display_door_and_handle(self, id):
        door_marker = Marker()
        door_marker.header.frame_id = "odom"
        door_marker.header.stamp = rospy.Time.now()
        door_marker.ns = "door positions"
        door_marker.id = 2
        door_marker.action = 0 # add/modify
        door_marker.type = 10 #mesh resource
        door_marker.mesh_resource = "package://dr_phil_gazebo//models//door-model/Door.dae"
        door_marker.pose.position.x = 3.183 
        door_marker.pose.position.y = -4.09
        #door_marker.pose.position.z = 3
        door_marker.pose.orientation.w = 1
        door_marker.color.a = 1
        door_marker.color.b= 1
        door_marker.color.r= 1
        door_marker.color.g= 1
        door_marker.scale.x = 0.001
        door_marker.scale.y = 0.001
        door_marker.scale.z = 0.001
        door_marker.lifetime = rospy.Duration(10)
        door_marker.mesh_use_embedded_materials = True
        return door_marker


    def visualise_spray_direction(self,origin_poses, end_poses):
        arrows = MarkerArray()
        for i in range(0,len(origin_poses.poses)):
            arrow = [origin_poses.poses[i].position, end_poses.poses[i].position] 
            point_spray = Marker()
            point_spray.header.frame_id = self.robot_frame
            point_spray.header.stamp = rospy.Time.now()
            point_spray.id = 0
            point_spray.type = 0 # arrow
            point_spray.action = 0 # add/modify
            point_spray.points = arrow
            point_spray.pose.orientation.w = origin_poses.poses[i].orientation.w
            point_spray.pose.orientation.x = origin_poses.poses[i].orientation.x
            point_spray.pose.orientation.y = origin_poses.poses[i].orientation.y
            point_spray.pose.orientation.z = origin_poses.poses[i].orientation.z
            point_spray.color.a = 0.5
            point_spray.color.g = 1 
            point_spray.scale.x = 0.01
            point_spray.scale.y = 0.01
            point_spray.scale.z = 0.01
            point_spray.ns = "Goal-%u"%i
            point_spray.lifetime = rospy.Duration(10)
            
            arrows.markers.append(point_spray) 
        return arrows
        

    


    
    def spin(self):
        if self.handle3D and self.normal:
            print(self.handle3D)
            self.visualise()



       

def main(data):
    
    SprayPath = SprayPathVisualiser()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        SprayPath.spin()
        rate.sleep()
   
    cv2.destroyAllWindows()


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)

    main(myargv)


   
