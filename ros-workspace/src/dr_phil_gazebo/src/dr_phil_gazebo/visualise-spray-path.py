#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import resource_retriever as Retriever
import rospy

from calc_spray_path import main as calculate_spray_end_points
from calc_spray_path import Coord

#from dr_phil_hardware.vision.localisation import *
#from dr_phil_hardware.vision.vision_handle_axis_algorithm import define_handle_features_heursitic
 


class SprayPathVisualiser:
    def __init__(self, spray_data):
        points = []
        spray_direction = []
        for data in spray_data:
            point = Point()
            point.x = (data[0])  #in meters
            point.y = (data[1])  #in meters
            point.z = (data[2])  #in meters
            end_point = Point()
            end_point.x = data[3] #in meters
            end_point.y = data[4] #in meters
            end_point.z = data[5] #in meters
            points.append(point)
            spray_direction.append(end_point)
        
  
        rospy.init_node('spray_path_visualiser',anonymous=True)
        spray_topic = 'spray_path_visualisation'
        self.vis_pub = rospy.Publisher(spray_topic, Marker, queue_size=100)
        rospy.sleep(2)
        point_camera = self.configurate_rviz_marker(points)
        self.vis_pub.publish(point_camera)
        
        door_estimated_topic = 'door_visualisation'
        self.door_pub = rospy.Publisher(door_estimated_topic, Marker, queue_size=10)
        rospy.sleep(2)
        door_marker = self.display_door_and_handle()
        self.door_pub.publish(door_marker)
        
        
        spray_direction_topic = 'spray_direction_visualisation'
        self.vis_arrow_pub = rospy.Publisher(spray_direction_topic, MarkerArray, queue_size=100)
        rospy.sleep(2)
        path_direction = self.visualise_spray_direction(points,spray_direction)
        self.vis_arrow_pub.publish(path_direction)
        
 
    def configurate_rviz_marker(self,points):
        point_camera = Marker()
        point_camera.header.frame_id = "camera_rgb_frame"
        point_camera.header.stamp = rospy.Time.now()
        point_camera.ns = "spray positions"
        point_camera.id = 0
        point_camera.type = 8 # sphere list
        point_camera.action = 0 # add/modify
        point_camera.points = points
        point_camera.pose.orientation.w = 1
        point_camera.color.a = 1
        point_camera.color.b = 1
        point_camera.scale.x = 0.01
        point_camera.scale.y = 0.01
        point_camera.scale.z = 0.01
        point_camera.lifetime = rospy.Duration(10)
        return point_camera
        
     
    def display_door_and_handle(self):
        door_marker = Marker()
        door_marker.header.frame_id = "odom"
        door_marker.header.stamp = rospy.Time.now()
        door_marker.ns = "door positions"
        door_marker.id = 0
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

    def visualise_spray_direction(self,points, end_points):
        arrows = MarkerArray()
        for i in range(0,len(points)):
            point_camera = Marker()
            arrow = [] 
            arrow.append(points[i])
            arrow.append(end_points[i])
            point_camera.header.frame_id = "camera_rgb_frame"
            point_camera.header.stamp = rospy.Time.now()
            point_camera.id = 0
            point_camera.type = 0 # arrow
            point_camera.action = 0 # add/modify
            point_camera.points = arrow
            point_camera.pose.orientation.w = 1
            point_camera.color.a = 0.5
            point_camera.color.g = 1 
            point_camera.scale.x = 0.01
            point_camera.scale.y = 0.01
            point_camera.scale.z = 0.01
            point_camera.ns = "Goal-%u"%i
            point_camera.lifetime = rospy.Duration(10)
            
            arrows.markers.append(point_camera) 
        return arrows
        


def main(data):
    if data is not None:
        SprayPath = SprayPathVisualiser(data)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    Center = Coord(0.0, 0.0, 0.0)
    Direction = Coord(1.0, 0.0, 0.0)

    data = calculate_spray_end_points(Center,Direction)
    main(data)
