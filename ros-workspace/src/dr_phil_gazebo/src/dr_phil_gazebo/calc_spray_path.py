#!/usr/bin/env python
import math
import numpy as np

# CONSTANTS
DISTANCE_FROM_HANDLE = 100  # (mm)
HANDLE_DIMENSIONS = [19, 130, 37]  # width, height, depth (mm)
SPRAY_ANGLE = 30  # (degrees)
SPRAY_RADIUS = DISTANCE_FROM_HANDLE * math.tan(math.radians(SPRAY_ANGLE))  # (mm)
CIRCLE_EDGE = math.sqrt(2 * SPRAY_RADIUS * SPRAY_RADIUS) / 2  # (mm)
SPRAY_CONFIDENCE = 10  # (mm)
        
    
class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

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
    num_sprays = 2*int(math.ceil(HANDLE_DIMENSIONS[1]/(2*CIRCLE_EDGE)))

    spray_centroids = np.zeros(num_sprays)
    index = 0

    current_z = center_z - CIRCLE_EDGE
    while current_z < top:
        current_z += CIRCLE_EDGE * 2
        spray_centroids[index] = current_z
        index += 1

    current_z = center_z + CIRCLE_EDGE
    while current_z > bottom:
        current_z -= CIRCLE_EDGE * 2
        spray_centroids[index] = current_z
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


def main(handle_center, vector):
    data = None

    print("Calculating spray coordinates...")
    if vector.x == 0 and vector.z == 0 and vector.y == 0:
        print("ERROR: cannot have unit vector direction (0,0,0)")
        exit(1)
    else:
        data = get_coords_and_vectors(handle_center, vector, False)
    
    
    return data



if __name__ == '__main__':
    center = Coord(0.0, 0.0, 0.0)
    direction = Coord(1.0, 0.0, 0.0)

    main(center, direction)
