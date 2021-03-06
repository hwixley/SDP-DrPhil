import math
import numpy as np
import copy

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
            self.gradient = float("inf")
        else:
            self.gradient = self.vector.x / self.vector.z

        if vector.x >= 0:
            if vector.z >= 0:
                self.segment = 0
            else:
                self.segment = 3
        elif vector.x < 0:
            if vector.z >= 0:
                self.segment = 1
            else:
                self.segment = 2


# Line1 represents current axes and Line2 represents destination axes
def calc_rotation_angle(line1, line2):
    angle = math.atan(line2.gradient) - math.atan(line1.gradient)

    # Changes angle depending on whet
    if not ((line1.gradient < 0 and line2.gradient < 0) or (line1.gradient > 0 and line2.gradient > 0)):
        angle = math.pi - abs(angle)
        if not ((line1.segment == 1 and line2.segment == 3) or line1.segment > line2.segment):
            angle = -1 * angle

    return math.degrees(angle)


# Given the center point and height to spray we can calculate the spray centroids
def calc_y_spray_centroids(center_y):
    top = (center_y + HANDLE_DIMENSIONS[1] / 2) + SPRAY_CONFIDENCE
    bottom = (center_y - HANDLE_DIMENSIONS[1] / 2) - SPRAY_CONFIDENCE
    num_sprays = 2 * int(math.ceil(HANDLE_DIMENSIONS[1] / (2 * CIRCLE_EDGE)))

    spray_centroids = np.zeros(num_sprays)
    index = 0

    current_y = center_y
    while current_y < top:
        current_y += CIRCLE_EDGE
        spray_centroids[index] = current_y
        index += 1

    current_y = center_y
    while current_y > bottom:
        current_y -= CIRCLE_EDGE
        spray_centroids[index] = current_y
        index += 1

    return spray_centroids


def calc_xz_spray_centroids(center, vector):
    xz_coords = np.empty((3, 2))
    xz_coords[0, :] = [center.x - DISTANCE_FROM_HANDLE, center.z]
    xz_coords[1, :] = [center.x, center.z - DISTANCE_FROM_HANDLE]
    xz_coords[2, :] = [center.x, center.z + DISTANCE_FROM_HANDLE]

    # Transform the points given the angle of rotation
    vector_line = LineGraph(vector)
    origin_line = LineGraph(ORIGIN)
    angle = calc_rotation_angle(origin_line, vector_line)
    print(angle)
    return xz_coords


def calc_coords(center, vector):
    xz_coords = calc_xz_spray_centroids(center, vector)
    y_coords = calc_y_spray_centroids(center.y)

    coords = np.empty((len(y_coords) * len(xz_coords), 3))

    row = 0
    for a in range(len(xz_coords)):
        for s in range(len(y_coords)):
            coords[row, 0] = xz_coords[a, 0]
            coords[row, 1] = y_coords[s]
            coords[row, 2] = xz_coords[a, 1]
            row += 1

    return coords


def main(center, vector):
    print("Calculating spray coordinates...")

    if vector.x == 0 and vector.z == 0 and vector.y == 0:
        print("ERROR: cannot have unit vector direction (0,0,0)")
        exit(1)
    else:
        coords = calc_coords(center, vector)
        print(coords)


if __name__ == '__main__':
    ORIGIN = Coord(0, 0, 0)

    center = Coord(0.0, 0.0, 0.0)

    direction = Coord(1.0, 0.0, -1.0)
    main(center, direction)

    direction = Coord(0.0, 0.0, 0.0)
    # main(center, direction)
