#!/usr/bin/env python3
import rospy
from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.ray import Ray
import numpy as np
from sensor_msgs.msg import CameraInfo
import tf
from geometry_msgs.msg import PointStamped,Point
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage

class HandleLocalizer:

    """ Node which listens to 2d points signifying features on the camera, and localizes them and then visualises them by publising
        a marker message
     """

    def __init__(self,feature_topic):


        self.robot_frame = "base_link"

        
        self.camera_info_sub = rospy.Subscriber("/camera_info",CameraInfo,callback=self.camera_info_callback)
        
        self.feature_sub = rospy.Subscriber(feature_topic,PointStamped,callback=self.feature_callback)

        self.feature_pub = rospy.Publisher("/handle_feature/camera_point",Marker,queue_size=10)

        self.camera = None

        self.point = None

        


    def camera_info_callback(self,camera_info : CameraInfo):
        


        self.camera = Camera(camera_info)    

        camera_transform_listener = tf.TransformListener()
        transformerRos = tf.TransformerROS()

        # this will block
        rospy.wait_for_message("/tf",TFMessage)


        # wait for transform to become available
        camera_transform_listener.waitForTransform(
            self.camera.get_frame_id(),
            self.robot_frame,
            rospy.Time.now(),
            timeout=rospy.Duration(2))

        # get transform
        (trans,rot) = camera_transform_listener.lookupTransform(
            self.camera.get_frame_id(),
            self.robot_frame,
            rospy.Time.now())
        

        try:
            transform = transformerRos.fromTranslationRotation(trans,rot)
            self.camera.setup_transform(transform)

        except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
            
            rospy.logerr("Could not find camera transform!")


        rospy.loginfo("Received camera info")

        # get rid of subscriber
        self.camera_info_sub.unregister()

    def feature_callback(self,point : PointStamped):
        self.point = point


    def visualise(self):
        
        camera_ray = self.camera.get_ray_through_image(np.array([self.point.point.x,self.point.point.y]))

        camera_ray_robot = self.camera.get_ray_in_robot_frame(camera_ray)
        print(camera_ray_robot)

        point_camera = Marker()
        point_camera.header.frame_id = self.robot_frame
        point_camera.header.stamp = rospy.Time.now()
        point_camera.ns = "handle_features"
        point_camera.id = 0
        point_camera.type = 0 # arrow
                
        p1 = Point()
        orig = camera_ray_robot.origin
        p1.x,p1.y,p1.z = (orig[0],orig[1],orig[2])
        
        p2 = Point()
        vec = camera_ray_robot.get_vec()
        p2.x,p2.y,p2.z = (camera_ray_robot.dir[0]+ orig[0],camera_ray_robot.dir[1]+ orig[1],camera_ray_robot.dir[2]+ orig[2]) 
        
        point_camera.points = [p1,p2]
        # point_camera.pose.position.x = self.point.point.x 
        # point_camera.pose.position.y = self.point.point.y 
        # point_camera.pose.position.z = 0
        point_camera.pose.orientation.w = 1
        point_camera.color.a = 1
        point_camera.color.b = 1
        point_camera.action = 0 # add/modify
        point_camera.scale.x = 0.01
        point_camera.scale.y = 0.02
        point_camera.scale.z = 0.05
        point_camera.lifetime = rospy.Duration(10)
        point_camera.frame_locked = True

        self.feature_pub.publish(point_camera)

    def spin(self):
        if self.camera and self.point:
            self.visualise()
        

    
if __name__ == "__main__":

    rospy.init_node("handle_localizer",anonymous=False)

    feature_topic = rospy.get_param("~feature_topic","/handle_feature")
    handleLocalizer = HandleLocalizer(feature_topic)


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        handleLocalizer.spin()
        rate.sleep()