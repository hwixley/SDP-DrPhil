#!/usr/bin/env python3

import numpy as np
import dr_phil_hardware.vision.utils as utils
from dr_phil_hardware.vision.ray import Ray
from image_geometry import PinholeCameraModel
import tf

import rospy

class Camera:

    def __init__(self,camera_info):
        """ 
            Args:
                camera_info: the camera_info message published by the camera 
        """
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(camera_info)

    def get_frame_id(self):
        return self.camera.tfFrame()

    def setup_transform(self,rob2cam):
        """
            sets up transform from camera to robot 
        """
        
        self.extrinsic_mat = rob2cam
        self.extrinsic_mat_inv = utils.invert_homog_mat(rob2cam)

        # for transforming directions
        self.extrinsic_mat_inv_no_trans = np.copy(self.extrinsic_mat_inv)
        self.extrinsic_mat_inv_no_trans[:-1,3]  = 0

    def get_pixel_through_ray(self,ray : Ray):
        """ assumes ray is going through origin of camera """

        # notice pixels are flipped
        (u,v) = self.camera.project3dToPixel(tuple(ray.get_vec()))
        return np.array([u,v])

    def get_ray_through_image(self,img_pos):
        """ returns the direction vector in camera space of the ray 
            passing through the camera center and all the 3D points corresponding to the given 2D point on the image  
        
            Args:
                img_pos (array_like): must be of length 2, the pixel coordinate (with origin in top left corner)
        """

        
        (u,v) = np.asarray(img_pos).flatten()
        ray_3d = self.camera.projectPixelTo3dRay((u,v))

        return Ray(np.array([[0],[0],[0]]),
            np.array(np.array(ray_3d).reshape((3,1))),length=1)

    def get_ray_in_robot_frame(self,ray : Ray) -> Ray: 
        """ transforms ray () from camera to robot space """
        
        dir = ray.dir
        dir = np.append(dir,np.array([[1]]),axis=0)

        # we rotate the direction but not translate it
        dir = (self.extrinsic_mat_inv_no_trans @ dir)[:-1,:]

        cam_origin_rf = self.extrinsic_mat_inv @ np.array([[0],[0],[0],[1]])

        ray = Ray(cam_origin_rf[:-1,:],dir,length=1)
          
        return ray

