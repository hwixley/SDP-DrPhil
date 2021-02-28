#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.camera import Camera
import dr_phil_hardware.vision.utils as utils
from dr_phil_hardware.vision.ray import Ray
import tf
from tf2_msgs.msg import TFMessage
import math

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
    
    def get_camera_ray_length(self, ray, data):
        """ returns a ray's length by averaging readings from the lidar sensor
            Args:
                ray: camera ray that must be in the plane formed by lidar rays
                data: lidar data
        """

        angle = data.angle_min
        while angle <= data.angle_max:
            lidar_ray1 = get_unit_vec_from_dir(angle)
            lidar_ray2 = get_unit_vec_from_dir((angle + data.angle_increment) % 360)
            subtr = subtract(lidar_ray1, lidar_ray2)
            
            if intersect(subtr, ray):
                return (data.ranges[i] + data.ranges[i + 1]) / 2
            
            angle += data.angle_increment

        return 0

    def get_normal_to_plane(self, l_ray1, l_ray2):
        """ returns the normal to the plane formed by the 2 lidar rays 
            endpoints with the normal to the ground
            Args:
                l_ray1: lidar ray
                l_ray2: lidar ray
        """
        v = np.subtract(l_ray1.origin, l_ray1.dir)
        normalized_v = v / np.sqrt(np.sum(v**2))
        l_ray1_endpoint = l_ray1.origin + normalized_v ** l_ray1.length

        u = np.subtract(l_ray2.origin, l_ray2.dir)
        normalized_u = u / np.sqrt(np.sum(u**2))
        l_ray2_endpoint = l_ray2.origin + normalized_u ** l_ray2.length

        horizontal_normal_endpoint = np.array([[0],[0],[1]]



    def get_unit_vec_from_dir(angle):
        """ return unit vector given an angle, counterclockwise from x-axis """
        origin = np.array([[0],[0],[0]]
        dir = np.array([[math.cos(angle)], [math.sin(angle)]])
        length = 1
        return Ray(origin, dir, length)