#!/usr/bin/env python3

import numpy as np
from ..utils import utils 
from .Ray import Ray

class Camera:

    def __init__(self, int_mat, r_to_cam_mat):

        # transforms points in camera frame to 2D Camera points
        # 3 x 4
        self.int_mat = int_mat

        # transforms points from camera coordinates to robot coordinates
        # 4x4
        self.r_to_cam_mat = r_to_cam_mat 

        # transforms points in robot coordinates to camera coordinates
        # 4 x 4
        self.cam_to_r_mat = utils.invert_homog_mat(r_to_cam_mat)

        # projects points in robot frame to 2D camera points
        # 3 x 4
        self.camera_mat_rf = int_mat @ r_to_cam_mat 
        
        # transforms 2D image points to rays through those points and camera center
        # 3 x 3
        self.pixel_to_ray_mat = np.linalg.inv(int_mat[0:,:-1])
        
        
    def get_ray_through_image(self,img_pos):
        """ returns the direction vector in camera space of the ray 
            passing through the camera center and all the 3D points corresponding to the given 2D point on the image  
        """
        ray_3d = self.pixel_to_ray_mat @ img_pos
        ray_3d /= -np.linalg.norm(ray_3d)
        return Ray(np.array([[0],[0],[0]]),ray_3d)

    def get_ray_in_robot_frame(self,ray):
        """ transforms ray () from camera to robot space """
        
        dir = ray.dir
        dir = np.append(dir,np.array([[1]]),axis=0)
        cam_origin_rf = self.cam_to_r_mat @ np.array([[0],[0],[0],[1]])

        return Ray(cam_origin_rf[:-1,:],((self.cam_to_r_mat[:,:-1] @ dir[:-1,:]) ))


class RasberryPiV2Camera(Camera):
    def __init__(self, r_to_cam_mat):
        pixel_width = 3280
        pixel_height = 2464
        mm_width = 3.68
        mm_height = 2.76

        f = 3.04 
        fx_px = (f / mm_width) * pixel_width
        fy_px = (f / mm_height) * pixel_height 
        cx = pixel_width/2
        cy = pixel_height/2

        intrinsic_matrix = np.array([[fx_px,0,    cx, 0],
                                    [0,    fy_px,cy, 0],
                                    [0,    0,    1,  0]])

        super().__init__(intrinsic_matrix,r_to_cam_mat)

def get_burget_bot_simple_camera():
    

    robot_to_cam_matrix = np.array([[-1,0, 0, 0],
                                    [ 0,1, 0, 0.103],
                                    [ 0,0,-1, 0.067],
                                    [ 0,0, 0, 1]])
    return RasberryPiV2Camera(robot_to_cam_matrix)

