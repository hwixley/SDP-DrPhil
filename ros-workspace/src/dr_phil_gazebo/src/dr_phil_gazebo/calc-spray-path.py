import math
import numpy as np
import copy

# CONSTANTS
DISTANCE_FROM_HANDLE = 100 # (mm)
HANDLE_DIMENSIONS = [19, 130, 37] # width, height, depth (mm)
SPRAY_ANGLE = 30 # (degrees)
SPRAY_RADIUS = DISTANCE_FROM_HANDLE*math.tan(math.radians(SPRAY_ANGLE)) # (mm)
CIRCLE_EDGE = math.sqrt(2*SPRAY_RADIUS*SPRAY_RADIUS)/2 # (mm)
SPRAY_CONFIDENCE = 10 # (mm)

class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class LineGraph:
    def __init__(self, vector):
        self.vector = vector
        self.gradient = self.direction.z/self.direction.x
        self.intercept = 0

        if vector.x >= 0 and vector.z >= 0:
            self.segment = 0
        elif vector.x <= 0 and vector.z >= 0:
            self.segment = 1
        elif vector.x <= 0 and vector.z <= 0:
            self.segment = 2
        else:
            self.segment = 3

#Line1 represents current axes and Line2 represents destination axes
def calcRotationAngle(line1, line2):
    angle = math.atan(line2.gradient) - math.atan(line1.gradient)

    # Denotes whether the angle is positive or negative
    if line1.segment == 1 line2.segment:
        angle = angle * -1

    return angle

# Given the center point and height to spray we can calculate the spray centroids
def calcYSprayCentroids(center_y):
    top = (center_y + HANDLE_DIMENSIONS[1]/2) + SPRAY_CONFIDENCE
    bottom = (center_y - HANDLE_DIMENSIONS[1]/2) - SPRAY_CONFIDENCE
    num_sprays = 2*int(math.ceil(HANDLE_DIMENSIONS[1]/(2*CIRCLE_EDGE)))

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

def calcXZSprayCentroids(center, vector):
    xz_coords = np.empty((3,2))
    xz_coords[0,:] = [center.x - DISTANCE_FROM_HANDLE, center.z]
    xz_coords[1,:] = [center.x, center.z - DISTANCE_FROM_HANDLE]
    xz_coords[2,:] = [center.x, center.z + DISTANCE_FROM_HANDLE]

    # Transform the points given the angle of rotation

    return xz_coords

def calcCoords(center, vector):
    xz_coords = calcXZSprayCentroids(center, vector)
    y_coords = calcYSprayCentroids(center.y)

    coords = np.empty((len(y_coords)*len(xz_coords), 3))

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
        coords = calcCoords(center, vector)
        print(coords)

if __name__ == '__main__':
    center = Coord(0.0, 0.0, 0.0)

    direction = Coord(1.0, 0.0, 0.0)
    main(center, direction)

    direction = Coord(0.0, 0.0, 0.0)
    #main(center, direction)

#Transform the coordinates based on the direction of the handle
#BASE CASE: (-1,y,0)
#if direction.x < 0 and direction.z == 0:
#        coords[:,0] = coords[:,0] * -1

#BASE CASE: (0,y,+-1)
#elif direction.x == 0:
#    if direction.z > 0:
#        x_col = copy.deepcopy(coords[:,0])
#        coords[:,0] = coords[:,2]
#        coords[:,2] = x_col * -1
#    else:
#        x_col = copy.deepcopy(coords[:,0])
#        coords[:,0] = coords[:,2] * -1
#        coords[:,2] = x_col

#GENERAL CASE: (x,y,z) where z != 0 and x != 0
#elif direction.x != 0 and direction.z != 0:
#    print("oops")