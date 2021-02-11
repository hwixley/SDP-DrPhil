import py_trees
import rospy
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

        super().__init__(name=name)    

        self.blackboard = py_trees.Blackboard()    
    
    def initialise(self):
        pass

    def update(self):
        if self.blackboard.get("scan") is not None:
            angle,distance = dummy_nearest_obstacle(self.blackboard.scan)
            self.blackboard.set("closest_obstacle/angle",angle)
            self.blackboard.set("closest_obstacle/distance",distance)

            self.feedback_message = str(angle) + ":" + str(distance)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No scan data"
            return py_trees.common.Status.FAILURE

class PublishTopic(py_trees.behaviour.Behaviour):
    """ a behaviour which publishes a certain message and always returning RUNNING on success"""

    def __init__(self,name,msg,msg_type,topic,queue_size=1,):
        """ 
        Args:
            name: the name of the behaviour
            msg_type: the type of message to be published
            topic: the topic on which to publish the message
            queue_size: the publisher queue size
        """

        super().__init__(name=name)
        self.msg = msg


        self.publisher = rospy.Publisher(topic,msg_type,queue_size=queue_size)
        

    def initialise(self):
        pass

    def update(self):
        self.feedback_message = "Waiting for data"
        try:
            self.feedback_message = "Published"
            self.publisher.publish(self.msg)
        except:
            self.feedback_message = "Publisher failure"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
       