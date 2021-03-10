#!/usr/bin/env python
import math
import numpy as np
import copy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import resource_retriever as Retriever
import rospy


# CONSTANTS
DISTANCE_FROM_HANDLE = 100  # (mm)
HANDLE_DIMENSIONS = [19, 130, 37]  # width, height, depth (mm)
SPRAY_ANGLE = 30  # (degrees)
SPRAY_RADIUS = DISTANCE_FROM_HANDLE * math.tan(math.radians(SPRAY_ANGLE))  # (mm)
CIRCLE_EDGE = math.sqrt(2 * SPRAY_RADIUS * SPRAY_RADIUS) / 2  # (mm)
SPRAY_CONFIDENCE = 10  # (mm)


class SprayPathVisualiser:
    def __init__(self, spray_data):
        points = []
        spray_direction = []
        for data in spray_data:
            point = Point()
            point.x = (data[0])  #in meters
            point.y = (data[1])  #in meters
            point.z = (data[2])  #in meters
            end_point = Point()
            end_point.x = data[3] *0.001 #in meters
            end_point.y = data[4] *0.001#in meters
            end_point.z = data[5] *0.001#in meters
            points.append(point)
            spray_direction.append(end_point)
        
  
        rospy.init_node('spray_path_visualiser',anonymous=True)
        spray_topic = 'spray_path_visualisation'
        self.vis_pub = rospy.Publisher(spray_topic, Marker, queue_size=100)
        rospy.sleep(2)
        point_camera = self.configurate_rviz_marker(points)
        self.vis_pub.publish(point_camera)
        
        door_estimated_topic = 'door_visualisation'
        self.door_pub = rospy.Publisher(door_estimated_topic, Marker, queue_size=10)
        rospy.sleep(2)
        door_marker = self.display_door_and_handle()
        self.door_pub.publish(door_marker)
        
        
        spray_direction_topic = 'spray_direction_visualisation'
        self.vis_arrow_pub = rospy.Publisher(spray_direction_topic, MarkerArray, queue_size=100)
        rospy.sleep(2)
        path_direction = self.visualise_spray_direction(points,spray_direction)
        self.vis_arrow_pub.publish(path_direction)
        
 
    def configurate_rviz_marker(self,points):
        point_camera = Marker()
        point_camera.header.frame_id = "camera_rgb_frame"
        point_camera.header.stamp = rospy.Time.now()
        point_camera.ns = "spray positions"
        point_camera.id = 0
        point_camera.type = 8 # sphere list
        point_camera.action = 0 # add/modify
        point_camera.points = points
        point_camera.pose.orientation.w = 1
        point_camera.color.a = 1
        point_camera.color.b = 1
        point_camera.scale.x = 0.01
        point_camera.scale.y = 0.01
        point_camera.scale.z = 0.01
        point_camera.lifetime = rospy.Duration(10)
        return point_camera
        
     
    def display_door_and_handle(self):
        door_marker = Marker()
        door_marker.header.frame_id = "odom"
        door_marker.header.stamp = rospy.Time.now()
        door_marker.ns = "door positions"
        door_marker.id = 0
        door_marker.action = 0 # add/modify
        door_marker.type = 10 #mesh resource
        door_marker.mesh_resource = "package://dr_phil_gazebo//models//door-model/Door.dae"
        door_marker.pose.position.x = 3.183 
        door_marker.pose.position.y = -4.09
        #door_marker.pose.position.z = 3
        door_marker.pose.orientation.w = 1
        door_marker.color.a = 1
        door_marker.color.b= 1
        door_marker.color.r= 1
        door_marker.color.g= 1
        door_marker.scale.x = 0.001
        door_marker.scale.y = 0.001
        door_marker.scale.z = 0.001
        door_marker.lifetime = rospy.Duration(10)
        door_marker.mesh_use_embedded_materials = True
        return door_marker

    def visualise_spray_direction(self,points, end_points):
        arrows = MarkerArray()
        for i in range(0,len(points)):
            point_camera = Marker()
            arrow = [] 
            arrow.append(points[i])
            arrow.append(end_points[i])
            point_camera.header.frame_id = "camera_rgb_frame"
            point_camera.header.stamp = rospy.Time.now()
            point_camera.id = 0
            point_camera.type = 0 # arrow
            point_camera.action = 0 # add/modify
            point_camera.points = arrow
            point_camera.pose.orientation.w = 1
            point_camera.color.a = 0.5
            point_camera.color.g = 1 
            point_camera.scale.x = 0.01
            point_camera.scale.y = 0.01
            point_camera.scale.z = 0.01
            point_camera.ns = "Goal-%u"%i
            point_camera.lifetime = rospy.Duration(10)
            
            arrows.markers.append(point_camera) 
        return arrows
        
        
    
class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z       


class LineGraph:
    def __init__(self, vector):
        self.vector = vector

        if self.vector.z == 0:
            self.gradient = float("inf")  # Vertical lines have infinity gradient
        else:
            self.gradient = self.vector.x / self.vector.z

        # Segments correspond to CAST plot and are used to determine the rotation direction
        if vector.x >= 0:
            if vector.z >= 0:
                self.segment = 0
            else:
                self.segment = 1
        elif vector.x < 0:
            if vector.z >= 0:
                self.segment = 3
            else:
                self.segment = 2


# Calculates the rotation angle (including direction) between two lines
# PARAMS: Line1 represents current axes, Line2 represents destination axes
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

    return math.degrees(angle)


# Transform the points given the angle of rotation
def transform_points(points, handle_center, vector):
    vector_line = LineGraph(vector)
    origin_line = LineGraph(Coord(0, 0, 0))
    angle = calc_rotation_angle(origin_line, vector_line)

    center_matrix = np.empty((2, 3))
    center_matrix[0, :] = handle_center.z
    center_matrix[1, :] = handle_center.x

    c, s = np.cos(angle), np.sin(angle)
    rotation_matrix = np.array(((c, -s), (s, c)))

    return np.matmul(rotation_matrix, points - center_matrix) + center_matrix


# Calculates y coordinates by starting at the center and going up and down the handle until the handle has been covered
def calc_y_spray_centroids(center_y):
    top = (center_y + HANDLE_DIMENSIONS[1] / 2) + SPRAY_CONFIDENCE
    bottom = (center_y - HANDLE_DIMENSIONS[1] / 2) - SPRAY_CONFIDENCE
    num_sprays = 2 * int(math.ceil(HANDLE_DIMENSIONS[1] / (2 * CIRCLE_EDGE)))

    spray_centroids = np.zeros(num_sprays)
    index = 0

    current_y = center_y - CIRCLE_EDGE
    while current_y < top:
        current_y += CIRCLE_EDGE*2
        spray_centroids[index] = current_y
        index += 1

    current_y = center_y + CIRCLE_EDGE
    while current_y > bottom:
        current_y -= CIRCLE_EDGE*2
        spray_centroids[index] = current_y
        index += 1

    return spray_centroids


# Calculates the xz locations around the handle for spraying
def calc_xz_spray_centroids(handle_center, vector):
    xz_coords = np.empty((2, 3))
    xz_coords[:, 0] = [handle_center.z, handle_center.x - DISTANCE_FROM_HANDLE]
    xz_coords[:, 1] = [handle_center.z - DISTANCE_FROM_HANDLE, handle_center.x]
    xz_coords[:, 2] = [handle_center.z + DISTANCE_FROM_HANDLE, handle_center.x]

    new_points = transform_points(xz_coords, handle_center, vector)

    return new_points


# Calculates the vectors which represent spray directions
def calc_vectors(vector):
    vectors = np.zeros((3, 3))
    vectors[0, :] = [vector.z, vector.x, 0]

    points = np.array([[vector.z], [vector.x]])
    center_matrix = np.array([[0], [0]])

    new_angles = [-90, 90]
    for i in range(2):
        angle = math.radians(new_angles[i])

        c, s = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array(((c, -s), (s, c)))

        spray_vector = np.matmul(rotation_matrix, points - center_matrix) + center_matrix
        vectors[i + 1, :] = [spray_vector[0, 0], spray_vector[1, 0], 0]

    return vectors


# Compiles xyz coordinates and their vectors into a matrix
def get_coords_and_vectors(handle_center, vector):
    xz_coords = calc_xz_spray_centroids(handle_center, vector)
    y_coords = calc_y_spray_centroids(handle_center.y)
    vectors = calc_vectors(vector)

    coords_and_vectors = np.empty(((len(y_coords) * xz_coords.shape[1]), 6))

    row = 0
    for a in range(xz_coords.shape[1]):
        for s in range(len(y_coords)):
            vector = DISTANCE_FROM_HANDLE * (vectors[a, :] / np.sum(abs(vectors[a, :])))  # Makes vector of magnitude DFH
            coords_and_vectors[row, 0] = xz_coords[0, a]
            coords_and_vectors[row, 1] = xz_coords[1, a]
            coords_and_vectors[row, 2] = y_coords[s]

            vector += coords_and_vectors[row, 0:3]  # Calculates the destination point of the vector
            coords_and_vectors[row, 3:6] = vector  # vectors[a, :] <-- old assignment
            row += 1

    coords_and_vectors[:, 0:3] = coords_and_vectors[:, 0:3]*0.001  # Convert to metres

    return coords_and_vectors


def main(handle_center, vector):
    print("Calculating spray coordinates...")

    if vector.x == 0 and vector.z == 0 and vector.y == 0:
        print("ERROR: cannot have unit vector direction (0,0,0)")
        exit(1)
    else:
        data = get_coords_and_vectors(handle_center, vector)
        print(data)
        
        visualiser = SprayPathVisualiser(data)
        
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    center = Coord(0.0, 0.0, 0.0)
    direction = Coord(1.0, 0.0, 0.0)

    main(center, direction)
