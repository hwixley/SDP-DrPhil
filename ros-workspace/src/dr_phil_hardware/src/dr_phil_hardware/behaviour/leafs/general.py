from __future__ import absolute_import
import py_trees
import numpy as np

def dummy_nearest_obstacle(scan):
    return(np.argmin(scan.ranges),0)

class ClosestObstacle(py_trees.behaviour.Behaviour):
    """ a behaviour which analyses the "/scan" blackboard variable and sets "closest_obstacle/angle" and "closest_obstacle/distance".
        returns FAILURE if no data available
    """

    def __init__(self,name):
        """
            Args:
                name: name of the behaviour
        """

        super(ClosestObstacle, self).__init__(name=name) 

        self.blackboard = py_trees.Blackboard()    
    
    def initialise(self):
        pass

    def update(self):
        if self.blackboard.get(u"scan") is not None:
            angle,distance = dummy_nearest_obstacle(self.blackboard.scan)
            self.blackboard.set(u"closest_obstacle/angle",angle)
            self.blackboard.set(u"closest_obstacle/distance",distance)

            self.feedback_message = unicode(angle) + u":" + unicode(distance)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = u"No scan data"
            return py_trees.common.Status.FAILURE
