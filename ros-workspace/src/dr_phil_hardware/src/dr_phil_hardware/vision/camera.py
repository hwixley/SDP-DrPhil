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
        print("\nfx:{},fy:{},cx:{},cy:{},Tx:{},Ty:{}".format(self.camera.fx(),self.camera.fy(),self.camera.cx(),self.camera.cy(),self.camera.Tx(),self.camera.Ty()))
        print("\nxz:{},yz{}".format(self.camera.cx()+self.camera.Tx(),self.camera.cy()+self.camera.Ty()))
    def get_frame_id(self):
        return self.camera.tfFrame()

    def setup_transform(self,rob2cam):
        """
            sets up transform from camera to robot 
        """
        
        self.extrinsic_mat = rob2cam
        self.extrinsic_mat_inv = utils.invert_homog_mat(rob2cam)

    def get_ray_through_image(self,img_pos):
        """ returns the direction vector in camera space of the ray 
            passing through the camera center and all the 3D points corresponding to the given 2D point on the image  
        
            Args:
                img_pos (array_like): must be of length 2, the pixel coordinate (with origin in top left corner)
        """
        (u,v) = np.asarray(img_pos).flatten()
        return Ray(np.array([[0],[0],[0]]),
            np.array(self.camera.projectPixelTo3dRay((u,v))).reshape((3,1)),length=1)

    def get_ray_in_robot_frame(self,ray : Ray) -> Ray: 
        """ transforms ray () from camera to robot space """
        
        dir = ray.dir
        dir = np.append(dir,np.array([[1]]),axis=0)
        dir = (self.extrinsic_mat_inv @ dir)
        dir = dir[:-1,:]

        cam_origin_rf = self.extrinsic_mat_inv @ np.array([[0],[0],[0],[1]])
        
        ray = Ray(cam_origin_rf[:-1,:],dir,length=1)
        return ray

