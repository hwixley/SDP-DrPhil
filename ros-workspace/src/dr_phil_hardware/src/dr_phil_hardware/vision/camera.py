#!/usr/bin/env python3

import numpy as np
import dr_phil_hardware.vision.utils as utils
from dr_phil_hardware.vision.ray import Ray
from image_geometry import PinholeCameraModel
import tf
import rospy
from tf2_msgs.msg import TFMessage

class Camera:

    def __init__(self,camera_info,robot_frame="/base_link"):
        """ 
            Args:
                camera_info: the camera_info message published by the camera 
        """
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(camera_info)
        
        self.camera_transform_listener = tf.TransformListener()
        self.tf_active_sub = rospy.Subscriber("/tf",TFMessage,self.setup_transform)
        self.transformerRos = tf.TransformerROS()

        
    def setup_transform(self,data):

        (trans,rot) = self.camera_transform_listener.lookupTransform(
            self.camera.tfFrame(),
            '/base_link',
            rospy.Time(0))

        try:
            transform = self.transformerRos.fromTranslationRotation(trans,rot)
        except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
            print("Could not find camera transform!")

        self.extrinsic_mat = transform
        self.extrinsic_mat_inv = utils.invert_homog_mat(transform)

        # unregister callback (run once only)
        self.tf_active_sub.unregister()

    def get_ray_through_image(self,img_pos):
        """ returns the direction vector in camera space of the ray 
            passing through the camera center and all the 3D points corresponding to the given 2D point on the image  
        
            Args:
                img_pos (array_like): must be of length 2, the pixel coordinate (with origin in top left corner)
        """
        (u,v) = np.asarray(img_pos).flatten()

        return Ray(np.array([[0],[0],[0]]),
            np.array(self.camera.projectPixelTo3dRay((u,v))).reshape((3,1)))

    def get_ray_in_robot_frame(self,ray):
        """ transforms ray () from camera to robot space """
        
        dir = ray.dir
        dir = np.append(dir,np.array([[1]]),axis=0)
        cam_origin_rf = self.extrinsic_mat_inv @ np.array([[0],[0],[0],[1]])

        ray = Ray(cam_origin_rf[:-1,:],((self.extrinsic_mat_inv[:,:-1] @ dir[:-1,:]) ))
        return ray

