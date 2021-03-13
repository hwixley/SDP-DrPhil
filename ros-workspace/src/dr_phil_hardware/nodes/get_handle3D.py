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
from geometry_msgs.msg import PointStamped,Point,PoseArray,Pose,PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from tf2_msgs.msg import TFMessage

#Library to process the image and try to find the bounding box of a handle
from dr_phil_hardware.ML.models import yolov3, DEFAULT_WEIGHTS, DEFAULT_CONFIGURATION, DEFAULT_OBJ_NAMES, visualise_results, load_network_and_classes


from dr_phil_hardware.vision.localisation import *
from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.lidar import Lidar
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.vision.localisation import localize_pixel
from dr_phil_hardware.vision.utils import invert_homog_mat, quat_from_yaw




from dr_phil_hardware.vision.vision_handle_axis_algorithm import define_handle_features_heursitic as get_center

"""
#The purpose of this node: 
(1) Take image and Run an Object Detection Model that helps us find handles and doors in an image
(2) Retrieve the important feature(s)/point(s) (in this case we are only interseted in the center of a pull door handle)
(3) Take the point(s) and transform it from coordinates in a 2D image to 3D coordinates with respect to the robot frame
(4) Publish the vector normal to a vertical surface (generated by the function localise_pixel) and the 3D center point of the handle
"""
class Handle3DTransformation:
  def __init__(self, weights, cfg):
        rospy.init_node('handle2D_to_3D',anonymous=True)
        self.weights = weights
        self.cfg = cfg


        self.robot_frame = "base_link"
        self.lidar_frame = "base_scan"
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

   
        # Stores the number of frames recieved and to be processed so far
        self.frame_id = 0
        #Store starting time to keep track of elapsed time
        self.starting_time = time.time()
        #Load the network and classes earlier - Done in order to improve efficiency of the node to process images quicker instead of taking time to load
        self.net, self.out, self.classes = load_network_and_classes(self.weights, self.cfg)
     
  
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        
        self.feature_pub = rospy.Publisher("/handle_feature/camera_points",MarkerArray,queue_size=10)

        self.scan_sub = rospy.Subscriber("/scan_filtered",LaserScan,callback=self.scan_callback)



        #Initialise image subscriber and call callback function
        self.image_sub = rospy.Subscriber("image", ImageMSG, self.yolo_callback)


        self.camera = None

        self.handle_box = None
        self.scan = None 


        #Initialise image publisher to send the calculated 3D world coordinates of handles from a camera image as well as normal to a vertical surface
        self.handle3D_pub = rospy.Publisher("/handle_feature/handle3D",Float64MultiArray,queue_size=10)
        self.normal_pub = rospy.Publisher("/handle_feature/normal",Float64MultiArray,queue_size=10)
        self.handle_pose_pub = rospy.Publisher("/handle_feature/pose",PoseStamped,queue_size=10)


              
   

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
              self.handle_box = box
        

        
        

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

  def visualise(self):

    point_np = np.array([[self.handle_box.x + (self.handle_box.width/2)],[self.handle_box.y + (self.handle_box.height/2)]])

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

    if point3d is not None and normal is not None: # normal and point3d are either None together or actual numbers
        #Publish handle point results as an array of numbers 
        handle3D_array = Float64MultiArray()
        handle3D_array.data = point3d
        self.handle3D_pub.publish(handle3D_array)

        #Publish normal vector results as array of numbers

        normal_array = Float64MultiArray()
        normal_array.data = normal.dir
        self.normal_pub.publish(normal_array)

        # publish handle pose (facing inwards)
        hps = PoseStamped()
        hps.header.frame_id = self.robot_frame
        hp = Pose()
        hps.pose = hp

        hp.position.x,hp.position.y,hp.position.z = point3d[0:3]
        hp.orientation.x,hp.orientation.y,hp.orientation.z,hp.orientation.w = quat_from_yaw(
            -angle_between_pi(
                normal.get_vec(),
                np.array([[1],[0],[0]]),
                plane_normal=np.array([[0],[0],[1]])))

        self.handle_pose_pub.publish(hps)

    
      

  def spin(self):
      if self.camera and self.handle_box and self.scan:
          self.visualise()
      



    
# call the class
def main(args):
    rospy.logdebug("Running handle2D_to_3D node...")
    weights = ""
    cfg = ""
    try:
        weights = args[1]
        cfg = args[2]

    except:
        weights = DEFAULT_WEIGHTS
        cfg = DEFAULT_CONFIGURATION
        if len(args)>1:
            rospy.logerr("If you want to use other weights and cfg (make sure they're the suitable cfg file for the weights), please add them to the Yolo Folder inside ML and run the node with the following format:")
            rospy.logerr("rosrun dr-phil <filename-in-ML>.weights <filename-in-ML-folder>.cfg")
            rospy.logerr("Default model was trained on two classes: (0) handle ; (1) door - See obj.names")
        
        rospy.loginfo("Using default weights and configuraton...")
    
    rospy.loginfo("weights: " + weights)
    rospy.loginfo("cfg: " + cfg)
    rospy.loginfo("Classes names contained in: " + DEFAULT_OBJ_NAMES)
    camera_parser = Handle3DTransformation(weights,cfg)
    
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
                camera_parser.spin()
                rate.sleep()
    except:
        cv2.destroyAllWindows()
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)