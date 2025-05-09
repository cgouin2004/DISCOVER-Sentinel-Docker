

import numpy as np

class BaseGeometry:
    def __init__(self):
        pass



class SpatialPosition(BaseGeometry):
    def __init__(self, x, y, z):
        super().__init__()
        self.x = x
        self.y = y
        self.z = z


    def as_list(self):
        return [self.x, self.y, self.z]

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


    def distance_to(self, pose):
        return np.linalg.norm(np.array(self.as_list()) - np.array(pose.as_list()))



    #TODO add other required methods, cartezian to others...