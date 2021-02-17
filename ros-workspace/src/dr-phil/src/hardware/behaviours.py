from numpy.core.fromnumeric import std
import py_trees
from py_trees.common import Status
import rospy
import numpy as np
import subprocess
import os 
import signal

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
        self.feedback_message = ""
    
    def update(self):

        process = self.blackboard.get(self.subprocess_variable_name)
        if process is not None:

            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            (out,err) = process.communicate()
            self.blackboard.set(self.subprocess_variable_name,None)

            self.feedback_message = "killed process: {0}".format(process)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "no process at {0} not found!".format(self.subprocess_variable_name)
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

        self.feedback_message = "starting process"

        # clear existing subprocesses
        
        self.kill_subprocess()

    def update(self):
        # check if there is already a subprocess running, i.e. if we started it
        launch_process = self.blackboard.get(self.subprocess_variable_name)
        # if already running, check on it untill watch time is over
        if launch_process is not None:            
            time_now = rospy.get_time()
            time_left = self.watch_time - (time_now - self.time_start)
            if time_left > 0:
                self.feedback_message = "watching process for {0}s ".format(str(time_left))

                # check on the process
                ok = self.is_subprocess_ok()

                if ok:
                    return py_trees.Status.RUNNING
                else:
                    return py_trees.Status.FAILURE

            else:
                self.feedback_message = "successfully started process"

                # we've babysat long enough
                return py_trees.Status.SUCCESS
        else:
            # start a subprocess
            self.start_subprocess()


            return py_trees.Status.RUNNING

    def terminate(self,newState):
        # we only need to clean up if we are interrupted in the middle 
        # of starting the process, in which case, just halt it, don't want hanging processes
        # if newState == py_trees.Status.INVALID:
        #     self.kill_subprocess()
        pass

    def kill_subprocess(self):
        """ kills subprocess started if it was started, otherwise does nothing """

        # find process
        launch_process = self.blackboard.get(self.subprocess_variable_name)
        
        # if it exists kill it and clear the blackboard
        if launch_process is not None:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)

            self.blackboard.set(self.subprocess_variable_name,None)

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

        FNULL = open(os.devnull, 'w')
        
        launch_process = subprocess.Popen(command, 
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT, shell=True,preexec_fn=os.setsid)
        self.blackboard.set(self.subprocess_variable_name,launch_process)

    def is_subprocess_ok(self):
        """ returns False if subprocess died"""

        launch_process = self.blackboard.get(self.subprocess_variable_name)

        state = launch_process.poll()

        if state is None:
            # ok
            return True

        elif state == 0:
            return True
        else:
            # terminated with error
            (out,_) = launch_process.communicate()
            self.feedback_message = "process interrupted with state of: {0}".format(state)
            self.logger.warning("process interrupted, stdout +err is: {0}".format(out))
            return False

            
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

    def __init__(self,name,msg,msg_type,topic,queue_size=1,success_on_publish=False,success_after_n_publishes=None):
        """ 
        Args:
            name: the name of the behaviour
            msg_type: the type of message to be published
            topic: the topic on which to publish the message
            queue_size: the publisher queue size
            success_on_publish: when true, will return SUCCESS after publishing each time
            success_after_n_publishes: when set to any integer, will return success after publishing n times without failure
        """

        super().__init__(name=name)
        self.msg = msg


        self.publisher = rospy.Publisher(topic,msg_type,queue_size=queue_size)
        self.success_on_publish = success_on_publish    
        self.n_target = -1 if success_after_n_publishes is None else success_after_n_publishes

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

        if self.success_on_publish or self.n_target >= 0:
            self.n_target -= 1
            if self.n_target > 0:
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING       