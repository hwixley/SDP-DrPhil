import py_trees
import rospy
import numpy as np
import subprocess
import os 

def dummy_nearest_obstacle(scan):
    return(np.argmin(scan.ranges),0)

class KillSubprocess(py_trees.behaviour.Behaviour):
    """ kills a subprocess previously started by some node stored on the blackboard, either returns SUCCESS if there process existed and was successfully killed, and FAILURE otherwise """

    def __init__(self, name, subprocess_variable_name="/subprocess"):
        """
            Args:
                name: name of behaviour
                subprocess_variable_name: the blackboard variable name containing subprocess to kill
        """
    
        super().__init__(name=name)
        self.blackboard = py_trees.Blackboard()
        self.subprocess_variable_name = subprocess_variable_name

    def initialise(self):
        pass
    
    def update(self):
        process = self.blackboard.get(self.subprocess_variable_name)

        if process is not None:
            process.kill()
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

class RunRos(py_trees.behaviour.Behaviour):
    """ Attempts to start up a launch/node file, returns SUCCESS if successfully started a program and FAILURE otherwise, will wait for the given amount of time to see if the child 
        terminates early and return RUNNING in this time.
        On success, saves the subprocess object to blackboard on the given variable_name.
        Should only be run once, if run while subprocess is running will return failure.
    """
    def __init__(self,name,launch_file=None,node_file=None,package="dr-phil",subprocess_variable_name="/subprocess", watch_time=0,args=[]):
        """
            Args: 
                name: name of the behaviour
                launch_file: the full filename of the launchfile
                package: the package containing the launch file (DEFAULT: "dr-phil")
                subprocess_variable_name: the blackboard variable name under which to save subprocess following a SUCCESS
                watch_time: the time in seconds to "babysit" the process for before deciding it's launched successfully
        """
        super().__init__(name=name)
        self.subprocess_variable_name = subprocess_variable_name
        self.watch_time = watch_time
        self.package = package
        self.launch_file = launch_file
        self.node_file = node_file
        self.blackboard = py_trees.Blackboard()
        self.args = args

    def initialise(self):
        self.time_start = rospy.get_time()
        # clear existing subprocesses

        launch_process = self.blackboard.get(self.subprocess_variable_name)
        
        if launch_process is not None:
            launch_process.kill()
            self.blackboard.set(self.subprocess_variable_name,None)

    def update(self):
        # check if there is already a subprocess running, i.e. if we started it
        launch_process = self.blackboard.get(self.subprocess_variable_name)
        # if already running, check on it untill watch time is over
        if launch_process is not None:            
            time_now = rospy.get_time()
            time_left = self.watch_time - (time_now - self.time_start)
            if time_left > 0:
                self.feedback_message = "on watch - {0}(s) left".format(str(time_left))

                # check on the process
                ok = self.is_subprocess_ok()

                if ok:
                    return py_trees.Status.RUNNING
                else:
                    self.feedback_message = "startup failed"
                    return py_trees.Status.FAILURE

            else:
                self.feedback_message = "completed watch"

                # we've babysat long enough
                return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "starting launch process"
            # start a subprocess
            self.start_subprocess()

            self.feedback_message = "started launch subprocess"

            return py_trees.Status.RUNNING

    def terminate(self,newState):
        pass # TODO: make sure this is fine empty
    
    def start_subprocess(self):

        file = self.launch_file if self.node_file is None else self.node_file

        main_cmd = "roslaunch" if self.node_file is None else "rosrun"
        
        arg = " "

        for k in self.args:
            arg += "{0} ".format(k)

        command = "{0} {1} {2} {3}".format(
            main_cmd,
            self.package,
            file,
            arg)
        print(command)

        FNULL = open(os.devnull, 'w')
        
        launch_process = subprocess.Popen(command, shell=True,
            stdout=FNULL,
            stderr=FNULL)

        self.blackboard.set(self.subprocess_variable_name,launch_process)

    def is_subprocess_ok(self):
        """ returns False if subprocess died"""

        launch_process = self.blackboard.get(self.subprocess_variable_name)

        state = launch_process.poll()
        if state is None:
            # ok
            return True
        elif state < 0:
            # terminated with error
            return False
        else:
            # terminated without error
            return True

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
       