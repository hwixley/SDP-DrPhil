

class Ray():
    def __init__(self,origin, dir):
        self.origin = origin
        self.dir = dir
    def __repr__(self):
        return "Ray:"+str(self.origin) + "\n->\n" + str(self.dir) + "\n"
