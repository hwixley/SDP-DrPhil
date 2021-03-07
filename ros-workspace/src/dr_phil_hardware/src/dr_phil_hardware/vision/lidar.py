#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.camera import Camera
import dr_phil_hardware.vision.utils as utils
from dr_phil_hardware.vision.ray import Ray
import tf
from tf2_msgs.msg import TFMessage
import math
import rospy 

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

    def get_ray_in_lidar_frame(self, ray : Ray):
        """ transforms ray() from robot to lidar space """
        
        return ray.get_transformed(self.extrinsic_mat_inv)
    
    def get_ray_projection(self, ray):
        """ returns projected ray in the same plane as lidar's rays (flatten) """
        """ set z component of direction to 0"""
        origin = ray.origin
        dir = ray.dir
        dir[-1] = 0
        return Ray(origin, dir)
    
    def get_corresponding_lidar_rays(self, camera_ray, data):
        """ returns the 2 corresponding lidar rays to a camera ray
            given in the lidar frame
            Args:
                camera_ray: camera ray in lidar frame
                data: LaserScan data
        """
        angle = data.angle_min
        
        # will have a problem if the angle increment is not an int when converted to deg 

        angle_deg = np.rad2deg(angle)
        while angle <= data.angle_max:
            lidar_ray1 = self.get_unit_vec_from_dir(angle)
            lidar_ray2 = self.get_unit_vec_from_dir((angle + data.angle_increment) % (2*math.pi))
            subtr = utils.subtract(lidar_ray1, lidar_ray2)
            
            if utils.intersect(subtr, camera_ray):
                lidar_ray1.length = data.ranges[angle_deg]
                lidar_ray2.length = data.ranges[(angle_deg + np.rad2deg(data.angle_increment)) % 360]
                return lidar_ray1, lidar_ray2
            
            angle += data.angle_increment
            angle_deg = np.rad2deg(angle)

        return None, None

    def get_camera_ray_length(self, camera_ray, data):
        """ returns a ray's length by averaging readings from the lidar sensor
            Args:
                camera_ray: camera ray that must be in the plane formed by lidar rays
                data: lidar data
        """

        lidar_ray1, lidar_ray2 = self.get_corresponding_lidar_rays(camera_ray, data)
        if lidar_ray1 != None:
            return (lidar_ray1.length + lidar_ray2.length) / 2
        else:
            return None

    def get_normal_to_plane(self, l_ray1, l_ray2):
        """ returns the normal to the plane formed by the 2 lidar rays 
            endpoints with the normal to the ground
            Args:
                l_ray1: lidar ray
                l_ray2: lidar ray
        """

        horizontal_normal_endpoint = np.array([[0],[0],[1]])

        v = l_ray1.dir
        normalized_v = v.dir / np.sqrt(np.sum(v.dir**2))
        l_ray1_endpoint = l_ray1.origin + normalized_v ** l_ray1.length

        u = l_ray2.dir
        normalized_u = u.dir / np.sqrt(np.sum(u.dir**2))
        l_ray2_endpoint = l_ray2.origin + normalized_u ** l_ray2.length

        subtr = np.subtract(l_ray1_endpoint, l_ray2_endpoint)
        normalized_subtr = subtr.dir / np.sqrt(np.sum(subtr.dir**2))
        halved_subtr_origin = subtr.origin + (normalized_subtr * subtr.length) / 2
        halved_subtr_dir = halved_subtr_origin + (normalized_subtr * subtr.length) / 2
        halved_subtr = Ray(halved_subtr_origin, halved_subtr_dir, subtr.length / 2)

        normal_dir = np.cross(halved_subtr.dir, horizontal_normal_endpoint)
        normal_origin = halved_subtr.origin
        normal_length = 1
        
        # normal_dir might need to be * -1 
        scalar_proj_of_normal_on_v = np.dot(normal_dir - normal_origin, v) / np.linalg.norm(v)
   
        if scalar_proj_of_normal_on_v[0] > 0:
            normal_dir *= -1

        return Ray(normal_origin, normal_dir, normal_length)



    def get_unit_vec_from_dir(self, angle):
        """ return unit vector given an angle in rad, counterclockwise from x-axis """
        origin = np.array([[0],[0],[0]])
        dir = np.array([[math.cos(angle)], [math.sin(angle)], [0]])
        length = 1
        return Ray(origin, dir, length)