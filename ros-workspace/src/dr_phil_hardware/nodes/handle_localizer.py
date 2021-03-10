#!/usr/bin/env python3
import rospy
from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.lidar import Lidar
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.vision.localisation import localize_pixel
from dr_phil_hardware.vision.utils import invert_homog_mat

import numpy as np
from sensor_msgs.msg import CameraInfo, LaserScan
import tf
from geometry_msgs.msg import PointStamped,Point
from visualization_msgs.msg import Marker,MarkerArray
from tf2_msgs.msg import TFMessage

class HandleLocalizer:

    """ Node which listens to 2d points signifying features on the camera, and localizes them and then visualises them by publising
        a marker message
     """

    def __init__(self,feature_topic):


        self.robot_frame = "base_link"
        self.lidar_frame = "base_scan"
        
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        
        self.feature_sub = rospy.Subscriber(feature_topic,PointStamped,callback=self.feature_callback)

        self.feature_pub = rospy.Publisher("/handle_feature/camera_points",MarkerArray,queue_size=10)

        self.scan_sub = rospy.Subscriber("/scan_filtered",LaserScan,callback=self.scan_callback)

        self.camera = None

        self.point = None
        self.scan = None 
        
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

    def feature_callback(self,point : PointStamped):
        self.point = point

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
        
        point_np = np.array([[self.point.point.x],[self.point.point.y]])

        camera_ray = self.camera.get_ray_through_image(point_np)

        camera_ray_robot = self.camera.get_ray_in_robot_frame(camera_ray)
        

        markers = MarkerArray()

        camera_mrkr = self.create_arrow_from_ray(camera_ray_robot,0)

        (point3d,normal) = localize_pixel(point_np,self.camera,self.lidar,self.scan,self.cam2lid)
        point3dmrkr = self.create_point_from_vec(point3d,1) 
        
        markers.markers = [camera_mrkr,point3dmrkr]

        self.feature_pub.publish(markers)

    def spin(self):
        if self.camera and self.point and self.scan:
            self.visualise()
        

    
if __name__ == "__main__":

    rospy.init_node("handle_localizer",anonymous=False)

    feature_topic = rospy.get_param("~feature_topic","/handle_feature")
    handleLocalizer = HandleLocalizer(feature_topic)


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        handleLocalizer.spin()
        rate.sleep()