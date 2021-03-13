#!/usr/bin/env python
import math
import numpy as np
import rospy 
import  tf.transformations as t
from geometry_msgs.msg import PointStamped,Point,PoseArray,Pose


from dr_phil_hardware.vision.localisation import *
from dr_phil_hardware.vision.utils import angle_between,angle_between_pi


"""
Note: Current implementation is in regards to Pull door handles only. Further extensibillity would need different ways to tackle different handles
or the possibility of implementing a dynamic solution
"""
# CONSTANTS
DISTANCE_FROM_HANDLE = 100  # (mm)
HANDLE_DIMENSIONS = [19, 130, 37]  # width, height, depth (mm)
SPRAY_ANGLE = 30  # (degrees)
SPRAY_RADIUS = DISTANCE_FROM_HANDLE * math.tan(math.radians(SPRAY_ANGLE))  # (mm)
CIRCLE_EDGE = math.sqrt(2 * SPRAY_RADIUS * SPRAY_RADIUS) / 2  # (mm)
SPRAY_CONFIDENCE = 0#10  # (mm)
        
    
class Coord:
    def __init__(self, x, y, z):
        self.x = x * 1000 #converted to mm
        self.y = y * 1000
        self.z = z * 1000 

class LineGraph:
    def __init__(self, vector):
        self.vector = vector

        if self.vector.x == 0:
            self.gradient = float("inf")  # Vertical lines have infinity gradient
        else:
            self.gradient = self.vector.y / self.vector.x

        # Segments correspond to CAST plot and are used to determine the rotation direction
        if vector.y >= 0:
            if vector.x >= 0:
                self.segment = 0
            else:
                self.segment = 1
        elif vector.y < 0:
            if vector.x >= 0:
                self.segment = 3
            else:
                self.segment = 2


# Line1 represents current axes and Line2 represents destination axes
def calc_rotation_angle(line1, line2):
    angle = math.atan(line2.gradient) - math.atan(line1.gradient)

    if not ((line1.segment in [0, 3] and line2.segment in [0, 3]) or (
            line1.segment in [1, 2] and line2.segment in [1, 2])) \
            or (line1.segment != line2.segment and angle == 0):
        angle = math.pi - angle

    if not ((line1.segment == line2.segment and line1.gradient < 0 and line1.gradient < line2.gradient) or \
            (line1.segment == line2.segment and line1.gradient > 0 and line1.gradient > line2.gradient) or \
            (line1.segment == 3 and line2.segment == 0) or (line1.segment < line2.segment)):
        angle = -1 * abs(angle)

    if line2.segment in [1, 2]:
        angle = 2 * math.pi - angle

    return angle


# Transform the points given the angle of rotation
def transform_points(points, handle_center, vector):
    vector_line = LineGraph(vector)
    origin_line = LineGraph(Coord(0, 1, 0))
    angle = calc_rotation_angle(origin_line, vector_line)

    center_matrix = np.empty((2, 3))
    center_matrix[0, :] = handle_center.x
    center_matrix[1, :] = handle_center.y

    c, s = math.cos(angle), math.sin(angle)
    rotation_matrix = np.array(((c, -s), (s, c)))

    return np.dot(rotation_matrix, points - center_matrix) + center_matrix


# Given the center point and height to spray we can calculate the spray centroids
def calc_z_spray_centroids(center_z):
    top = (center_z + HANDLE_DIMENSIONS[1]/2) + SPRAY_CONFIDENCE
    bottom = (center_z - HANDLE_DIMENSIONS[1]/2) - SPRAY_CONFIDENCE
    num_sprays = int(math.ceil(HANDLE_DIMENSIONS[1]/(2*CIRCLE_EDGE)))

    spray_centroids = np.zeros(num_sprays)
    index = 0

    current_z = center_z + CIRCLE_EDGE
    while current_z < top and index < num_sprays:
        spray_centroids[index] = current_z
        current_z += CIRCLE_EDGE * 2
        index += 1

    current_z = center_z - CIRCLE_EDGE
    while current_z > bottom and index < num_sprays:
        spray_centroids[index] = current_z
        current_z -= CIRCLE_EDGE * 2
        index += 1

    return spray_centroids


def calc_xy_spray_centroids(handle_center, vector):
    xy_coords = np.empty((2, 3))
    xy_coords[:, 0] = [handle_center.x, handle_center.y - DISTANCE_FROM_HANDLE]
    xy_coords[:, 1] = [handle_center.x - DISTANCE_FROM_HANDLE, handle_center.y]
    xy_coords[:, 2] = [handle_center.x + DISTANCE_FROM_HANDLE, handle_center.y]

    new_points = transform_points(xy_coords, handle_center, vector)

    return new_points


# NOT USED FOR RVIZ: calculates the vectors which represent spray directions
def calc_vectors(vector):
    vectors = np.zeros((3, 3))
    vectors[0, :] = [vector.x, vector.y, 0]

    points = np.array([[vector.x], [vector.y]])
    center_matrix = np.array([[0], [0]])

    new_angles = [-90, 90]
    for i in range(2):
        angle = math.radians(new_angles[i])

        c, s = math.cos(angle), math.sin(angle)
        rotation_matrix = np.array(((c, -s), (s, c)))

        spray_vector = np.dot(rotation_matrix, points - center_matrix) + center_matrix
        vectors[i + 1, :] = [spray_vector[0, 0], spray_vector[1, 0], 0]

    return vectors


def get_coords_and_vectors(handle_center, vector, output_is_vector):
    xy_coords = calc_xy_spray_centroids(handle_center, vector)
    z_coords = calc_z_spray_centroids(handle_center.z)

    coords_and_vectors = np.empty(((len(z_coords) * xy_coords.shape[1]), 6))

    if output_is_vector:
        vectors = calc_vectors(vector)
        row = 0
        for a in range(xy_coords.shape[1]):
            for s in range(len(z_coords)):
                coords_and_vectors[row, 0] = xy_coords[0, a]
                coords_and_vectors[row, 1] = xy_coords[1, a]
                coords_and_vectors[row, 2] = z_coords[s]
                coords_and_vectors[row, 3:6] = vectors[a, :]

                row += 1
    else:
        coords_and_vectors[:, 3] = handle_center.x
        coords_and_vectors[:, 4] = handle_center.y

        row = 0
        for a in range(xy_coords.shape[1]):
            for s in range(len(z_coords)):
                coords_and_vectors[row, 0] = xy_coords[0, a]
                coords_and_vectors[row, 1] = xy_coords[1, a]
                coords_and_vectors[row, 2] = z_coords[s]
                coords_and_vectors[row, 5] = z_coords[s]

                row += 1

    coords_and_vectors[:, 0:6] = coords_and_vectors[:, 0:6] * 0.001  # Convert to metres

    return coords_and_vectors

"""
Currently returns values of type PoseArray()

Given a handle in center and the normal to a vertical surface pointing away from handle, get the positions of the points
that an arm needs to reach, forming an inverted T shape to try to cover all handles. 
Orientation in quaternions denotes the direction of the spray from a position to point at the handle.

Returns two variables of type PoseArray() from geometry_msgs.msg in ROS, with 
"""
def get_position_and_orientation_of_spray_points(handle3D,normal_vector, robot_frame):
    Center = Coord(handle3D[0], handle3D[1], handle3D[2])
    Direction = Coord(normal_vector[0], normal_vector[1], normal_vector[2])

    #Calculate points and appropriate "end points"/ directions
    spray_data = calculate_spray_path(Center,Direction)
    origin_points = []
    spray_direction = []
    #We want to convert the direction vectors we have to represent orientation. 
    #This will be done by taking the transformation from spray points to the direction pointed by end points in the vector AB
    #To further illustrate, origin_points are considered OA which are the points around the handles, spray direction as AB to represent vector 
    #pointing to center of handle (**only similar in one axis, doesnt actually point to straight into the center**), where OB is the center of handle.
    poses = []
    end_poses = []
    if spray_data is not None:
        for data in spray_data:
            #Calculate the orientation ; data[0:3] represents the points beside the handles and data[3:6] representing the "end" points or points on the same vertical line as pull-door handle

            spray_ray = Ray(data[0:3,None],data[3:6,None] - data[0:3,None],length=1)

            thetaHandle =- angle_between_pi(spray_ray.get_vec(), np.array([[1],[0],[0]]),plane_normal=np.array([[0],[0],[1]]))
            # TODO: for some reason one of the sides on the T points pi/4 rads away from handle center

            orientationHandle = t.quaternion_from_matrix(t.rotation_matrix(thetaHandle,np.array([0,0,1])))
            #Target points for arm to reach to
            point_pose = Pose()
            point = Point()
            point.x = spray_ray.origin[0] #in meters
            point.y = spray_ray.origin[1] #in meters
            point.z = spray_ray.origin[2] #in meters
            point_pose.position = point
            point_pose.orientation.x = orientationHandle[0]
            point_pose.orientation.y = orientationHandle[1]
            point_pose.orientation.z = orientationHandle[2]
            point_pose.orientation.w = orientationHandle[3]

            #Endpoint - Spray direction 
            end_point_pose = Pose()
            end_point = Point()
            end_point.x = data[3] #in meters
            end_point.y = data[4] #in meters
            end_point.z = data[5] #in meters
            end_point_pose.position = end_point
            end_point_pose.orientation.x = 0
            end_point_pose.orientation.y = 0
            end_point_pose.orientation.z = 0
            end_point_pose.orientation.w = 1


            #Append pose results in a list to 
            origin_points.append(point)
            poses.append(point_pose)
            spray_direction.append(end_point)
            end_poses.append(end_point_pose)
        #Store the results in PoseArray
        #data[0:3]
        spray_origin_poses = PoseArray()
        spray_origin_poses.header.frame_id = robot_frame
        spray_origin_poses.header.stamp = rospy.Time.now()
        spray_origin_poses.poses = poses
        #data[3:6]
        spray_endpoints_poses = PoseArray()
        spray_endpoints_poses.header.frame_id = robot_frame
        spray_endpoints_poses.header.stamp = rospy.Time.now()
        spray_endpoints_poses.poses = end_poses

        return spray_origin_poses, spray_endpoints_poses

        

        
    
    return None, None




def calculate_spray_path(handle_center, vector):
    data = None
    
    print("Calculating spray coordinates...")
    if vector.x == 0 and vector.z == 0 and vector.y == 0:
        print("ERROR: cannot have unit vector direction (0,0,0)")
        exit(1)
    else:
        vector = Coord(vector.x*-1, vector.y*-1, vector.z)
        data = get_coords_and_vectors(handle_center, vector, False)

    return data


#Testing
if __name__ == '__main__':
    center = Coord(0.0, 0.0, 0.0)
    direction = Coord(1.0, 0.0, 0.0)

    calculate_spray_path(center, direction)
