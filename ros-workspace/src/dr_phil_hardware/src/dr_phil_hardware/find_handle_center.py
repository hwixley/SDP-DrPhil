#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge, CvBridgeError
import sys
import time
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Float64MultiArray
import tf
from geometry_msgs.msg import PointStamped,Point,PoseArray,Pose
from visualization_msgs.msg import Marker,MarkerArray
from tf2_msgs.msg import TFMessage

from dr_phil_hardware.ML.models import yolov3, DEFAULT_WEIGHTS, DEFAULT_CONFIGURATION, DEFAULT_OBJ_NAMES, visualise_results, load_network_and_classes


from dr_phil_hardware.vision.localisation import *
from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.lidar import Lidar
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.vision.localisation import localize_pixel
from dr_phil_hardware.vision.utils import invert_homog_mat





#Library to calculate points around handle To Move Arm to (to cover all handle)
from dr_phil_hardware.calc_spray_path import main as calculate_spray_end_points
from dr_phil_hardware.calc_spray_path import Coord


#from dr_phil_hardware.vision.vision_handle_axis_algorithm import define_handle_features_heursitic

class TestYOLO:
  def __init__(self, weights, cfg):
        rospy.init_node('find_handle_center',anonymous=True)
        self.weights = weights
        self.cfg = cfg
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

   
        # Stores the number of frames recieved and to be processed so far
        self.frame_id = 0
        #Store starting time to keep track of elapsed time
        self.starting_time = time.time()
        #Load the network and classes earlier - Done in order to improve efficiency of the node to process images quicker instead of taking time to load
        self.net, self.out, self.classes = load_network_and_classes(self.weights, self.cfg)
     
  

        self.robot_frame = "camera_rgb_frame"
        self.lidar_frame = "base_scan"
        
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        
        #self.feature_sub = rospy.Subscriber(feature_topic,PointStamped,callback=self.feature_callback)

        self.feature_pub = rospy.Publisher("/handle_feature/camera_points",MarkerArray,queue_size=10)

        self.scan_sub = rospy.Subscriber("/scan_filtered",LaserScan,callback=self.scan_callback)

        self.spray_origin_pub = rospy.Publisher("/spray_path/target_points",PoseArray,queue_size=10)
        self.spray_endpoints_pub = rospy.Publisher("/spray_path/direction_points",PoseArray,queue_size=10)



        self.camera = None

        self.point = None
        self.scan = None 



        #Initialise image subscriber and call callback function
        self.image_sub = rospy.Subscriber("image", ImageMSG, self.yolo_callback)

        


        
              
   

  def yolo_callback(self,rgb_msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            #Increment the number of frames to be processed
            self.frame_id += 1
        except CvBridgeError as e:
            print(e)

        #Pass the image and the appropriate arguments to get results
        results = yolov3(self.rgb_image,weights=self.weights,cfg=self.cfg, network=self.net, output_layers=self.out, class_names=self.classes)
        
        
        #See the YOLO results by calling the visualise results function
        visualise_results(self.rgb_image, results, self.starting_time, self.frame_id)

        #Filter the results to have handles only - for now
        for box in results:
            #Get the first handle result
            if box.label=="handle":
              self.point = box
        

        
        

  def scan_callback(self,scan : LaserScan):
      self.scan = scan 

  def camera_info_callback(self,camera_info : CameraInfo):
      
      self.camera = Camera(camera_info)    

      transform_listener = tf.TransformListener()
      transformerRos = tf.TransformerROS()

      # this will block
      rospy.wait_for_message("/tf",TFMessage)


      # wait for camera transform to become available
      transform_listener.waitForTransform(
          self.camera.get_frame_id(),
          self.robot_frame,
          rospy.Time.now(),
          timeout=rospy.Duration(2))

      # get transform
      (trans,rot) = transform_listener.lookupTransform(
          self.camera.get_frame_id(),
          self.robot_frame,
          rospy.Time.now())

      try:
          self.rob2cam = transformerRos.fromTranslationRotation(trans,rot)
          self.camera.setup_transform(self.rob2cam)
      except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
          rospy.logerr("Could not find camera transform!")


      self.lidar = Lidar()
      # wait for lidar transform to become available

      transform_listener.waitForTransform(
          self.lidar_frame,
          self.robot_frame,
          rospy.Time.now(),
          timeout=rospy.Duration(2))

      # get lidar transform
      (trans,rot) = transform_listener.lookupTransform(
          self.lidar_frame,
          self.robot_frame,
          rospy.Time.now())

      try:
          self.rob2lid = transformerRos.fromTranslationRotation(trans,rot)
          self.lidar.setup_transform(self.rob2lid)
      except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
          rospy.logerr("Could not find camera transform!")

      # chain transforms to get cam2lid
      self.cam2lid = self.rob2lid @ invert_homog_mat(self.rob2cam)

      rospy.loginfo("Received camera info")

      # get rid of subscriber
      self.camera_info_sub.unregister()

  def create_spray_path_points(self,handle_3d_vector, normal,camera_ray):
    x,y,z = handle_3d_vector[0,0], handle_3d_vector[1,0], handle_3d_vector[2,0]
    Center = Coord(x*1000, y*1000, z*1000)
    Direction = Coord(normal.dir[0]*1000,normal.dir[1]*1000,normal.dir[2]*1000)

    spray_data = calculate_spray_end_points(Center,Direction)
    #Convert them to poseArray
    print(spray_data)
    origin_points = []
    spray_direction = []

    x_unit= np.array([[0],[0],[0]])
    thetaHandle = angle_between_pi(camera_ray.get_vec(),normal.get_vec())
    orientationHandle = tf.transformations.quaternion_from_matrix(tf.transformations.rotation_matrix(thetaHandle,normal.get_vec()))
    self.spray_origin_poses = PoseArray()
    self.spray_endpoints_poses = PoseArray()
    poses = []
    end_poses = []

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
        end_point_pose.orientation.x = orientationHandle[0]
        end_point_pose.orientation.y = orientationHandle[1]
        end_point_pose.orientation.z = orientationHandle[2]
        end_point_pose.orientation.w = orientationHandle[3]


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




    
    return origin_points,spray_direction



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
      door_marker.header.frame_id = "base_link"
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
    

  def create_point_from_vec(self,vec,id):
      pnt = Marker()

      pnt.header.frame_id = self.robot_frame
      pnt.header.stamp = rospy.Time.now()

      pnt.ns = "handle_features"
      pnt.id = id
      pnt.type = 2 # sphere
      pnt.pose.orientation.w = 1
      pnt.color.a = 1
      pnt.color.b = 1
      pnt.action = 0 # add/modify
      pnt.scale.x = 0.05
      pnt.scale.y = 0.05
      pnt.scale.z = 0.05
      pnt.lifetime = rospy.Duration(10)
      pnt.frame_locked = True
      print(vec)
      pnt.pose.position.x = vec[0,0]
      pnt.pose.position.y = vec[1,0]
      pnt.pose.position.z = vec[2,0]

      return pnt 

  def create_arrow_from_ray(self,ray,id):
      arw = Marker()

      arw.header.frame_id = self.robot_frame
      arw.header.stamp = rospy.Time.now()

      arw.ns = "handle_features"
      arw.id = id
      arw.type = 0 # arrow

      p1 = Point()
      orig = ray.origin
      p1.x,p1.y,p1.z = (orig[0],orig[1],orig[2])
      
      p2 = Point()
      p2.x,p2.y,p2.z = (ray.dir[0]+ orig[0],ray.dir[1]+ orig[1],ray.dir[2]+ orig[2]) 

      arw.points = [p1,p2]
      arw.pose.orientation.w = 1
      arw.color.a = 1
      arw.color.b = 1
      arw.action = 0 # add/modify
      arw.scale.x = 0.01
      arw.scale.y = 0.02
      arw.scale.z = 0.05
      arw.lifetime = rospy.Duration(10)
      arw.frame_locked = True

      return arw

  def display_door_and_handle(self,id):
      door_marker = Marker()
      door_marker.header.frame_id = "odom"
      door_marker.header.stamp = rospy.Time.now()
      door_marker.ns = "door positions"
      door_marker.id = id
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

  def visualise(self):
        
    point_np = np.array([[self.point.x + (self.point.width/2)],[self.point.y + (self.point.height/2)]])

    camera_ray = self.camera.get_ray_through_image(point_np)

    camera_ray_robot = self.camera.get_ray_in_robot_frame(camera_ray)
    camera_ray_lidar = self.lidar.get_ray_in_lidar_frame(camera_ray_robot)

    markers = MarkerArray()

    camera_mrkr = self.create_arrow_from_ray(camera_ray,0)
    camera_mrkr.header.frame_id = self.camera.get_frame_id()
    camera_lidar_mrkr = self.create_arrow_from_ray(camera_ray_lidar,1)
    camera_lidar_mrkr.color.r = 1
    camera_lidar_mrkr.header.frame_id = self.lidar_frame

    (point3d,normal) = localize_pixel(point_np,self.camera,self.lidar,self.scan)

    if point3d is not None:
      spray_origin, spray_endpoints = self.create_spray_path_points(point3d, normal, camera_ray_robot)



    #Publish pose results
    self.spray_origin_pub.publish(self.spray_origin_poses)
    self.spray_endpoints_pub.publish(self.spray_endpoints_poses)



    

    if point3d is not None:
        point3dmrkr = self.create_point_from_vec(point3d,2) 
        normal3dmrkr = self.create_arrow_from_ray(normal,3)
        markers.markers = [camera_mrkr,camera_lidar_mrkr,point3dmrkr,normal3dmrkr]
    else:
        markers.markers = [camera_mrkr,camera_lidar_mrkr]

    self.feature_pub.publish(markers)

    print(point3d)

    door_estimated_topic = 'door_visualisation'
    self.door_pub = rospy.Publisher(door_estimated_topic, Marker, queue_size=10)
    door_marker = self.display_door_and_handle(2)
    self.door_pub.publish(door_marker)


    spray_topic = 'spray_path_visualisation'
    self.vis_pub = rospy.Publisher(spray_topic, Marker, queue_size=100)
    rospy.sleep(2)
    point_camera = self.configurate_rviz_marker(spray_origin)
    self.vis_pub.publish(point_camera)
  
    
    spray_direction_topic = 'spray_direction_visualisation'
    self.vis_arrow_pub = rospy.Publisher(spray_direction_topic, MarkerArray, queue_size=100)
    rospy.sleep(2)
    path_direction = self.visualise_spray_direction(spray_origin,spray_endpoints)
    self.vis_arrow_pub.publish(path_direction)

    
    print("all good")



    

      

  def spin(self):
      if self.camera and self.point and self.scan:
          self.visualise()
      



    
# call the class
def main(args):
  print("Running Node...")
  print()
  weights = ""
  cfg = ""
  try:
      weights = args[1]
      cfg = args[2]
  except:
    weights = DEFAULT_WEIGHTS
    cfg = DEFAULT_CONFIGURATION
    print("If you want to use other weights and cfg (make sure they're the suitable cfg file for the weights), please add them to the Yolo Folder inside ML and run the node with the following format:")
    print("rosrun dr-phil <filename-in-ML>.weights <filename-in-ML-folder>.cfg")
    print("Default model was trained on two classes: (0) handle ; (1) door - See obj.names")
    print()
    print("Using default weights and configuraton...")


      
  print("weights: " + weights)
  print("cfg: " + cfg)
  print("Classes names contained in: " + DEFAULT_OBJ_NAMES)
  camera_parser = TestYOLO(weights,cfg)
  
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
        camera_parser.spin()
        rate.sleep()
   
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)
