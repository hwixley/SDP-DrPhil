from numpy.lib.function_base import angle
from dr_phil_hardware.vision.camera import Camera
from dr_phil_hardware.vision.lidar import Lidar
from dr_phil_hardware.vision.ray import Ray
from dr_phil_hardware.vision.utils import interpolated_ray,angle_between,invert_homog_mat

from sensor_msgs.msg import LaserScan
import numpy as np
import math 

def localize_pixel(img_pos,camera : Camera,lidar : Lidar, scan : LaserScan, cam2lid) -> tuple:
    """ given 2d image, lidar and camera as well as the current scan message, localizes the pixel against the lidar data 
    
        Args:
            img_pos: 2x1 numpy array of pixel coordinates
            camera: the initialised camera object
            lidar: the initialized lidar object
            scan: the current scan data to localize against 
            cam2lid: the tf transform from camera to lidar (numpy version)
        Returns:
            tuple[np.array,np.array]: the 3d point in robot space of the pixel and the normal to the lidar intersection plane at that point (flattened to horizontal)
                        if no scan data corresponds to the point, returns (None,None)
    """

#       ---OBJ--
#    x  r1 /\ r2 x
#         /  \
#cam_ray /    \ average_ray
#       /      \
#      /        \
#     CAM ----> LID
#     

    # has to be 2d
    assert (img_pos.size == 2)

    cam_ray = camera.get_ray_through_image(img_pos)
    cam_ray_robot = camera.get_ray_in_robot_frame(cam_ray)
    cam_ray_lidar = lidar.get_ray_in_lidar_frame(cam_ray_robot)

    # flatten camera ray
    cam_ray_lidar_flat = lidar.get_ray_projection(cam_ray_lidar)

    # figure out which lidar rays correspond to the camera ray
    (ray1,ray2) = lidar.get_corresponding_lidar_rays(cam_ray_lidar_flat,scan)

    # if no rays found corresponding to scan data
    if ray1 is None or ray2 is None:
        return (None,None)

    # get the normal to the lidar hit
    intersection_normal = lidar.get_normal_to_plane(ray1,ray2)

    # get the distance data in horizontal plane, from lidar to object
    lidar_to_target_length = lidar.get_camera_ray_length(cam_ray_lidar_flat,scan)

    # get the vector from camera to lidar (flattened to lidar plane)
    # i.e. origin of lidar frame in camera frame
    lidar_offset = (cam2lid @ np.array([[0],[0],[0],[1]]))[:-1,:]
    cam_origin_lidar = (invert_homog_mat(cam2lid) @ np.array([[0],[0],[0],[1]]))[:-1,:]
    cam_to_lidar_flat = lidar.get_ray_projection(Ray(cam_origin_lidar,lidar_offset,np.linalg.norm(lidar_offset)))

    # now workout the lidar to object ray, i.e. interpolate between ray1's and ray2's tips
    lidar_to_object = interpolated_ray(ray1,ray2,0.5,lidar_to_target_length)

    # now finally workout the vector from camera to object (flattened)
    # this lets us access the true z-distance in the camera
    cam_to_object_flat = lidar_to_object.get_vec() + cam_to_lidar_flat.get_vec()
    cam_to_object_flat_length = np.linalg.norm(cam_to_object_flat)

    # angle from horizontal on camera ray
    cam_ray_theta = angle_between(cam_ray.get_vec(),cam_to_object_flat)

    # length of original camera ray (knowing the length of its projection)
    # will fail if ray is pointing straight up or down
    cam_ray.length = cam_to_object_flat_length / math.cos(cam_ray_theta)

    # we know simply transform the lidar to robot frame, giving us the point we need
    cam_ray_robot = lidar.get_ray_in_robot_frame(cam_ray)    

    object_robot = cam_ray_robot.get_vec()+cam_ray_robot.origin

    return (object_robot,intersection_normal)