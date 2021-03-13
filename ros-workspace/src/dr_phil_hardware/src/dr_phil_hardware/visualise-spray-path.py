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
#Library to process the image and try to find the bounding box of a handle
# from models import yolov3, DEFAULT_WEIGHTS, DEFAULT_CONFIGURATION, DEFAULT_OBJ_NAMES, visualise_results, load_network_and_classes

#Library to calculate points around handle To Move Arm to (to cover all handle)
from calc_spray_path import main as calculate_spray_end_points
from calc_spray_path import Coord

from dr_phil_hardware.vision.localisation import *
from dr_phil_hardware.vision.vision_handle_axis_algorithm import define_handle_features_heursitic
 


class SprayPathVisualiser:
    def __init__(self):

        rospy.init_node('SprayPath',anonymous=True)
        self.handle3D = None
        self.normal = None

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


    def visualise(self):        
        spray_origin, spray_endpoints = self.create_spray_path_points()



        #TODO: Publish pose results!!! 
        self.spray_origin_pub.publish(self.spray_origin_poses)
        #self.spray_endpoints_pub.publish(self.spray_endpoints_poses)


        #Visualise markers and publish them for rviz to view them
        point_camera = self.configurate_rviz_marker(spray_origin)
        self.vis_pub.publish(point_camera)

        path_direction = self.visualise_spray_direction(spray_origin,spray_endpoints)
        self.vis_arrow_pub.publish(path_direction)

        door_marker = self.display_door_and_handle(2)
        self.door_pub.publish(door_marker)




        



    

    
    def create_spray_path_points(self,camera_ray=None):
        Center = Coord(self.handle3D[0], self.handle3D[1], self.handle3D[2])
        Direction = Coord(self.normal[0], self.normal[1], self.normal[1])

        spray_data = calculate_spray_end_points(Center,Direction)
        origin_points = []
        spray_direction = []

        #TODO: Calculate orientation handle
       
        #Convert them to poseArray
        self.spray_origin_poses = PoseArray()
        self.spray_endpoints_poses = PoseArray()
        poses = []
        end_poses = []
        if spray_data is not None:
            for data in spray_data:
                #Target points for arm to reach to
                point_pose = Pose()
                point = Point()
                point.x = (data[0])  #in meters
                point.y = (data[1])  #in meters
                point.z = (data[2])  #in meters
                point_pose.position = point
                point_pose.orientation.x = orientationHandle[0]
                point_pose.orientation.y = orientationHandle[1]
                point_pose.orientation.z = orientationHandle[2]
                point_pose.orientation.w = orientationHandle[3]

                #Endpoint - Spray direction 
                end_point_pose = Pose()
                end_point = Point()
                end_point.x = data[3] #in meters
                end_point.y = data[4] #in meters
                end_point.z = data[5] #in meters
                end_point_pose.position = end_point
                # end_point_pose.orientation.x = orientationHandle[0]
                # end_point_pose.orientation.y = orientationHandle[1]
                # end_point_pose.orientation.z = orientationHandle[2]
                # end_point_pose.orientation.w = orientationHandle[3]
                x_unit= np.array([[1],[0],[0]])
                thetaHandle = angle_between_pi(self.normal.get_vec(),)
                thetaHandle = angle_between_pi(np.array([self.normal[0],self.normal[1],self.normal[2]]), x_unit)
                orientationHandle = tf.transformations.quaternion_from_matrix(tf.transformations.rotation_matrix(thetaHandle,np.array([[self.normal[0],self.normal[1],self.normal[2]]))

                #Append results
                origin_points.append(point)
                poses.append(point_pose)
                spray_direction.append(end_point)
                end_poses.append(end_point_pose)

        self.spray_origin_poses.header.frame_id = self.robot_frame
        self.spray_origin_poses.header.stamp = rospy.Time.now()
        self.spray_origin_poses.poses = poses
        self.spray_endpoints_poses.header.frame_id = self.robot_frame
        self.spray_endpoints_poses.header.stamp = rospy.Time.now()
        self.spray_endpoints_poses.poses = poses




        #TODO: Return appropraite values - currently just returns the points and not the poses with orientation
        return origin_points,spray_direction


    #TODO: change ids 
 
    def configurate_rviz_marker(self,points):
        point_camera = Marker()
        point_camera.header.frame_id = self.robot_frame
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


    def visualise_spray_direction(self,points, end_points):
        arrows = MarkerArray()
        for i in range(0,len(points)):
            point_camera = Marker()
            arrow = [] 
            arrow.append(points[i])
            arrow.append(end_points[i])
            point_camera.header.frame_id = self.robot_frame
            point_camera.header.stamp = rospy.Time.now()
            point_camera.id = 0
            point_camera.type = 0 # arrow
            point_camera.action = 0 # add/modify
            point_camera.points = arrow
            point_camera.pose.orientation.w = self.spray_origin_poses.poses[i].orientation.w
            point_camera.pose.orientation.x = self.spray_origin_poses.poses[i].orientation.x
            point_camera.pose.orientation.y = self.spray_origin_poses.poses[i].orientation.y
            point_camera.pose.orientation.z = self.spray_origin_poses.poses[i].orientation.z
            point_camera.color.a = 0.5
            point_camera.color.g = 1 
            point_camera.scale.x = 0.01
            point_camera.scale.y = 0.01
            point_camera.scale.z = 0.01
            point_camera.ns = "Goal-%u"%i
            point_camera.lifetime = rospy.Duration(10)
            
            arrows.markers.append(point_camera) 
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

   
