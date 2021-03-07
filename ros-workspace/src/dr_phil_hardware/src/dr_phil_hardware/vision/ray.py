import math
import numpy as np

class Ray():
    def __init__(self,origin, dir, length = math.inf):
        self.origin = origin
        self.dir = dir / np.linalg.norm(dir)
        self.length = length

        if not (origin.size == dir.size):
            raise ValueError("The origin and direction have different dimensionality: {} and {}".format(origin.size,dir.size))

    def get_vec(self):
        """ returns ray as vector from origin in direction of the ray's length """
        return ((self.dir * self.length) + self.origin) - self.origin

    def __repr__(self):
        return "Ray:"+str(self.origin) + "\n->\n" + str(self.dir) + "\nLength: " + str(self.length) + "\n"
