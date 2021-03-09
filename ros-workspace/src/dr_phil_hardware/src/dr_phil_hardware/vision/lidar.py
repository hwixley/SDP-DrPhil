#!/usr/bin/env python3

import numpy as np
from dr_phil_hardware.vision.camera import Camera
import dr_phil_hardware.vision.utils as utils
from dr_phil_hardware.vision.ray import Ray
import tf
from tf2_msgs.msg import TFMessage
import math
import rospy 
from sensor_msgs.msg import LaserScan

class Lidar:

    def __init__(self):
        pass
    
    def setup_transform(self,rob2scan):

        self.extrinsic_mat = rob2scan
        self.extrinsic_mat_inv = utils.invert_homog_mat(rob2scan)


    def get_ray_in_robot_frame(self,ray:Ray):
        return ray.get_transformed(self.extrinsic_mat)

    def get_ray_in_lidar_frame(self, ray : Ray):
        """ transforms ray() from robot to lidar space """
        
        return ray.get_transformed(self.extrinsic_mat_inv)
    
    def get_ray_projection(self, ray : Ray):
        """ returns projected ray in the same plane as lidar's rays (flatten) """
        """ set z component of direction to 0"""

        #original_v = ray.get_vec()

        origin = ray.origin
        dir = ray.dir
        dir[-1] = 0

        new_v = origin + dir

        #dot = original_v @ new_v.T
        #new_length = np.linalg.norm(new_v)
        return Ray(origin, dir,length=1 )
    
    def get_corresponding_lidar_rays(self, camera_ray, data: LaserScan) -> tuple :
        """ returns the 2 corresponding lidar rays to a camera ray
            given in the lidar frame
            Args:
                camera_ray: camera ray in lidar frame
                data: LaserScan data
        """
        angle = data.angle_min

        camera_ray.length = data.range_max * 2

        # will have a problem if the angle increment is not an int when converted to deg 

        angle_deg = int(np.rad2deg(angle))
        while angle <= data.angle_max:
            lidar_ray1 = self.get_unit_vec_from_dir(angle)
            lidar_ray2 = self.get_unit_vec_from_dir((angle + data.angle_increment) % (2*math.pi))
            subtr = utils.subtract(lidar_ray1, lidar_ray2)
            
            if utils.intersect(subtr, camera_ray):
                lidar_ray1.length = data.ranges[angle_deg]
                lidar_ray2.length = data.ranges[(angle_deg + int(np.rad2deg(data.angle_increment))) % 360]
                return lidar_ray1, lidar_ray2
            
            angle += data.angle_increment
            angle_deg = int(np.rad2deg(angle))

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
        normalized_v = v / np.sqrt(np.sum(v**2))
        l_ray1_endpoint = l_ray1.origin + normalized_v ** l_ray1.length

        u = l_ray2.dir
        normalized_u = u / np.sqrt(np.sum(u**2))
        l_ray2_endpoint = l_ray2.origin + normalized_u ** l_ray2.length

        subtr = l_ray1_endpoint - l_ray2_endpoint
        normalized_subtr = subtr / np.linalg.norm(subtr)

        halved_subtr = subtr * 0.5 

        normal_dir = np.cross(halved_subtr, horizontal_normal_endpoint,axisa=0,axisb=0)
        normal_origin = l_ray2_endpoint
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
    
    def get_angle_from_unit_vec(self, ray: Ray):
        """ return angle made with Ox by vector converted to deg """
        return (int(np.rad2deg(math.atan2(ray.dir[1], ray.dir[0]))) + 360) % 360