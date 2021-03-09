import math
import numpy as np

class Ray():
    def __init__(self,origin, dir, length = math.inf):
        self.origin = origin
        self.dir = dir / np.linalg.norm(dir)
        self.length = length

        if not (origin.size == dir.size):
            raise ValueError("The origin and direction have different dimensionality: {} and {}".format(origin.size,dir.size))

    def get_point(self):
        """ converts the ray to a point, with origin at the rays origin  (i.e. in the frame of the ray)"""
        return ((self.dir * self.length) + self.origin)

    def get_vec(self):
        """ returns ray as vector from origin in direction of the ray's length """
        return self.get_point() - self.origin

    def __repr__(self):
        return "Ray:"+str(self.origin) + "\n->\n" + str(self.dir) + "\nLength: " + str(self.length) + "\n"


    def get_transformed(self,transformation):
        """ returns new transformed ray using the provided transformation """
        
        dir = self.dir
        dir = np.append(dir,np.array([[1]]),axis=0)

        # we rotate the direction but not translate it (i.e. dont use the translation column)
        dir = (transformation[:,:-1] @ dir[:-1,:])[:-1,:]

        cam_origin_rf = transformation @ np.array([[0],[0],[0],[1]])

        ray = Ray(cam_origin_rf[:-1,:],dir,length=1)

        return ray