import math

class Ray():
    def __init__(self,origin, dir, length = math.inf):
        self.origin = origin
        self.dir = dir
        self.length = length
    def __repr__(self):
        return "Ray:"+str(self.origin) + "\n->\n" + str(self.dir) + "\nLength: " + str(self.length) + "\n"
