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
     
  

        self.robot_frame = "base_link"
        self.lidar_frame = "base_scan"
        
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        
        self.feature_pub = rospy.Publisher("/handle_feature/camera_points",MarkerArray,queue_size=10)

        self.scan_sub = rospy.Subscriber("/scan_filtered",LaserScan,callback=self.scan_callback)




        #Initialise image subscriber and call callback function
        self.image_sub = rospy.Subscriber("image", ImageMSG, self.yolo_callback)


        self.camera = None

        self.point = None
        self.scan = None 


        #Initialise image publisher to send the calculated 3D world coordinates of handles from a camera image as well as normal to a vertical surface
        self.handle3D_pub = rospy.Publisher("/handle_feature/handle3D",Float64MultiArray,queue_size=10)
        self.normal_pub = rospy.Publisher("/handle_feature/normal",Float64MultiArray,queue_size=10)



              
   

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

    print(self.point)
    point_np = np.array([[self.point.x + (self.point.width/2)],[self.point.y + (self.point.height/2)]])
    print(point_np)

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
    print(point3d)

  

    if point3d is not None:
        point3dmrkr = self.create_point_from_vec(point3d,2) 
        normal3dmrkr = self.create_arrow_from_ray(normal,3)
        markers.markers = [camera_mrkr,camera_lidar_mrkr,point3dmrkr,normal3dmrkr]
    else:
        markers.markers = [camera_mrkr,camera_lidar_mrkr]

    self.feature_pub.publish(markers)

    print(point3d)

    if point3d is not None:
        #Publish handle point results as an array of numbers 
        handle3D_array = Float64MultiArray()
        handle3D_array.data = point3d
        self.handle3D_pub.publish(handle3D_array)
    #Publish normal vector results as array of numbers
    if normal is not None:
        normal_array = Float64MultiArray()
        normal_array.data = normal.dir
        self.normal_pub.publish(normal_array)


    
      

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
