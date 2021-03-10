import math
import numpy as np

class Ray():
    def __init__(self,origin, dir, length = math.inf):
        self.origin = origin

        dir_length = np.linalg.norm(dir)
        self.dir = dir / dir_length if dir_length else dir

        self.length = length

        if not (origin.size == dir.size):
            raise ValueError("The origin and direction have different dimensionality: {} and {}".format(origin.size,dir.size))

    def get_point(self):
        """ converts the ray to a point, with origin at the rays origin  (i.e. in the frame of the ray)
        
            Raises:
                ValueException: if length is not finite
        """
        if not math.isfinite(self.length):
            raise ValueError("Cannot convert infinite ray to point")

        return ((self.dir * self.length) + self.origin)

    def get_vec(self):
        """ returns ray as vector from origin in direction of the ray's length 
                
            Raises:
                ValueError: if length is not finite"""
        if not math.isfinite(self.length):
            raise ValueError("Cannot convert infinite ray to vector")

        return self.get_point() - self.origin 

    def __repr__(self):
        return "Ray:"+str(self.origin) + "\n->\n" + str(self.dir) + "\nLength: " + str(self.length) + "\n"


    def get_transformed(self,transformation):
        """ returns new transformed ray using the provided transformation """

        dir = self.dir
        
        # we rotate the direction but not translate it (i.e. dont use the translation column)
        dir = (transformation[:,:-1] @ dir)[:-1,:]

        origin_homog = np.append(self.origin,np.array([[1]]),axis=0)
        new_origin = transformation @ origin_homog

        ray = Ray(new_origin[:-1,:],dir,length=self.length)

        return ray