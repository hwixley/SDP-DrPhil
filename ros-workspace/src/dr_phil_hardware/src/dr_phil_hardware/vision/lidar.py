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
from tf.transformations import projection_matrix
class Lidar:

    def __init__(self):
        pass
    
    def setup_transform(self,rob2scan):

        self.extrinsic_mat = rob2scan
        self.extrinsic_mat_inv = utils.invert_homog_mat(rob2scan)

    def get_ray_in_robot_frame(self,ray:Ray):
        return ray.get_transformed(self.extrinsic_mat_inv)

    def get_ray_in_lidar_frame(self, ray : Ray):
        return ray.get_transformed(self.extrinsic_mat)
    
    def get_ray_projection(self, ray : Ray):
        """ returns projected ray in the same plane as lidar's rays (flatten) """
        """ set z component of direction and origin to 0"""

        v_proj = ray.get_vec()
        v_proj[-1,:] = 0
        proj_length = np.linalg.norm(v_proj)

        n_orig = ray.origin 
        n_orig[-1,:] = 0

        return Ray(n_orig, v_proj,length=proj_length )
    

    def is_invalid_range_reading(self,reading,data : LaserScan) -> bool:
        return reading == math.inf or reading < data.range_min or reading > data.range_max


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

            lidar_ray1.length = data.ranges[angle_deg]
            lidar_ray2.length = data.ranges[(angle_deg + int(np.rad2deg(data.angle_increment))) % 360]

            # we need to check if either of the rays is an invalid reading
            if not(self.is_invalid_range_reading(lidar_ray1.length,data) or self.is_invalid_range_reading(lidar_ray2.length,data)):
                subtr = utils.subtract(lidar_ray1, lidar_ray2)
                
                #print("\nCurrently checking: \nlray1 angle: {}\nlray2 angle: {}\n".format(self.get_angle_from_unit_vec(lidar_ray1), self.get_angle_from_unit_vec(lidar_ray2)))
                if utils.intersect(subtr, camera_ray):
                    #print("intersecting!!!")
                    return lidar_ray1, lidar_ray2
                
            angle += data.angle_increment
            angle_deg = int(np.rad2deg(angle))

        return None, None

    def get_camera_ray_length(self, camera_ray, lidar_ray1,lidar_ray2):
        """ returns a ray's length by averaging readings from the lidar sensor
            Args:
                camera_ray: camera ray that must be in the plane formed by lidar rays
                data: lidar data
        """

        if lidar_ray1 != None:
            return (lidar_ray1.length + lidar_ray2.length) / 2
        else:
            return None

    def get_normal_to_plane(self, l_ray1 : Ray, l_ray2 : Ray):
        """ returns the normal to the plane formed by the 2 lidar rays 
            endpoints with the normal to the ground. origin of the normal
            is in the middle of the 2 lidar rays endpoints
            Args:
                l_ray1: lidar ray
                l_ray2: lidar ray
        """

        subtr = utils.subtract(l_ray1, l_ray2)
        halved_subtr_origin = subtr.origin + 0.5*subtr.length*subtr.dir 

        normal_dir = np.cross(subtr.get_vec(), np.array([[0], [0], [1]]),axisa=0,axisb=0,axisc=0)

        # normal_dir might need to be * -1 
        scalar_proj_of_normal_on_ray = normal_dir.T @ l_ray2.get_vec() / l_ray2.length
        if scalar_proj_of_normal_on_ray > 0:
            normal_dir *= -1

        return Ray(halved_subtr_origin, normal_dir, 1)



    def get_unit_vec_from_dir(self, angle):
        """ return unit vector given an angle in rad, counterclockwise from x-axis """
        origin = np.array([[0], [0], [0]])
        dir = np.array([[math.cos(angle)], [math.sin(angle)], [0]])
        length = 1
        return Ray(origin, dir, length)
    
    def get_angle_from_unit_vec(self, ray: Ray):
        """ return angle made with Ox by vector converted to deg """
        return (int(np.rad2deg(math.atan2(ray.dir[1], ray.dir[0]))) + 360) % 360