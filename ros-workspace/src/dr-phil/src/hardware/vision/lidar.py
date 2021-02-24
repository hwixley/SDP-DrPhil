#!/usr/bin/env python3

import numpy as np
from vision.camera import Camera
import vision.utils as utils
from vision.ray import Ray
import tf
from tf2_msgs.msg import TFMessage

class Lidar:

    def __init__(self):
        self.robot_transform_listener = tf.TransformListener()
        self.tf_active_sub = rospy.Subscriber("/tf",TFMessage,self.setup_transform)
        self.transformerRos = tf.TransformerROS()
    
    def setup_transform(self,data):

        (trans,rot) = self.robot_transform_listener.lookupTransform(
            '/base_link',
            '/base_scan',
            rospy.Time(0))

        try:
            transform = self.transformerRos.fromTranslationRotation(trans,rot)
        except (tf.ConnectivityException,tf.LookupException,tf.ExtrapolationException):
            print("Could not find robot transform!")

        self.extrinsic_mat = transform
        self.extrinsic_mat_inv = utils.invert_homog_mat(transform)

        # unregister callback (run once only)
        self.tf_active_sub.unregister()

    def get_ray_in_lidar_frame(self, ray):
        """ transforms ray() from robot to lidar space """
        
        dir = ray.dir
        dir = np.append(dir,np.array([[1]]),axis=0)
        robot_origin_rf = self.extrinsic_mat_inv @ np.array([[0],[0],[0],[1]])

        ray = Ray(robot_origin_rf[:-1,:],((self.extrinsic_mat_inv[:,:-1] @ dir[:-1,:]) ))
        return ray
    
    def get_ray_projection(self, ray):
        """ returns projected ray in the same plane as lidar's rays (flatten) """
        """ set z component of direction to 0"""
        origin = ray.origin
        dir = ray.dir
        dir[-1] = 0
        return Ray(origin, dir)
    
    def get_ray_length(self, ray):
        """ returns a ray's length by averaging readings from the lidar sensor
            Args:
                ray: must be in the plane formed by lidar rays
        """
        self.input_sub = rospy.Subscriber('scan',LaserScan,self.callback) 


    def match_with_lidar_rays(self, ray):
        """ returns the 2 closest lidar rays to the ray passed as argument
            Args:
                ray: must be in the plane formed by lidar rays
        """

    def get_normal_to_plane(self, l_ray1, l_ray2):
        """ returns the normal to the plane formed by the 2 lidar rays 
            endpoints with the normal to the ground
            Args:
                l_ray1: lidar ray
                l_ray2: lidar ray
        """

    def callback(self, data):
        for (angle, distance) in enumerate(data, start = 0):
            for (nxt_angle, nxt_distance) in enumerate(data, start=1):
                

        # unregister callback (run once only)
        self.input_sub.unregister()