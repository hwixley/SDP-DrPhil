import threading
from numpy.core.fromnumeric import std
import py_trees
from py_trees.common import Status
import rospy
import numpy as np
import subprocess
import os 
import signal
from threading  import Thread
import sys

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty  # python 2.x

def dummy_nearest_obstacle(scan):
    return(np.argmin(scan.ranges),0)



class RunRos(py_trees.behaviour.Behaviour):
    """ Runs and returns RUNNING status for as long as given process is running, will return FAILURE whenever it halts, and if killed via blackboard kill key will return SUCCESS and
        won't reset untill this key is cleared (i.e. will keep returning SUCCESS ). Optionally can serve as short process launcher
    """
    def __init__(self,name,blackboard_alive_key,blackboard_kill_trigger_key,launch_file=None,node_file=None,package="dr-phil",args=[],success_on_non_error_exit=False):
        """
            Args: 
                name: name of the behaviour
                blackboard_alive_key: the key which will be set as long as the process is running
                blackboard_kill_trigger_key: the key which if set will cause the node to kill the process and keep returning SUCCESS as long as it's set (on the next tick of this node)
                launch_file: the full filename of the launchfile
                package: the package containing the launch file (DEFAULT: "dr-phil")
                success_on_non_error_exit: will not return FAILURE if process exits with successfull return code (i.e. completes), use for running single-use launchfiles/nodes
        """
        super().__init__(name=name)
        self.package = package
        self.launch_file = launch_file
        self.node_file = node_file
        self.blackboard = py_trees.Blackboard()

        self.alive_key = blackboard_alive_key
        self.kill_key = blackboard_kill_trigger_key
        self.success_on_non_error_exit = success_on_non_error_exit
        self.args = args

        self.process = None
        self.last_success = False 
        self.stdout_queue = None
        self.stderr_queue = None

    def initialise(self):


        # if we previously succeeded, i.e. either:
        # 1) another node killed us on purpose
        # 2) we run a single-use node/launchfile to completion
        if self.last_success:
            self.feedback_message = "completed" 
            if self.blackboard.get(self.kill_key) is None:
                self.last_success = False
                self.feedback_message = "re-starting process"
            else:
                self.feedback_message += " , kill key set, not-starting new processes"
                
        else: # if we were just started, or previously failed
            self.feedback_message = "starting process"
            
            # TODO: check this is not redundant cosnidering terminate() calls it too
            # AFAIK terminate will trigger a kill before each initialization anyway
            self.kill_subprocess()

    def update(self):

        # if we were killed successfully last time
        # keep returning SUCCESS
        if self.last_success:
            return py_trees.Status.SUCCESS

        # check if there is already a subprocess running, i.e. if we started it
        # if already running, check on it untill watch time is over
        if self.process is not None:          

            self.feedback_message= "running process"

            # check on the process
            ok = self.is_subprocess_ok()


            # log output of program as it's running
            self.subprocess_log_output()

            # if process is still running successfully
            if ok == 0:
                # check kill trigger
                if self.blackboard.get(self.kill_key) is not None:
                    # kill
                    self.kill_subprocess()
                    self.last_success = True

                    return py_trees.Status.SUCCESS

                return py_trees.Status.RUNNING

            # if process died when not expected
            elif ok == 2 or (ok == 1 and not self.success_on_non_error_exit):
                return py_trees.Status.FAILURE

            # if process died but we expected it
            else:
                return py_trees.Status.SUCCESS
                self.last_success = True

        else:
            # start a subprocess
            self.start_subprocess()


            return py_trees.Status.RUNNING

    def terminate(self,newState):

        # before we enter each initialize or after we are pre-empted
        # we make sure to not leave hanging processes
        self.kill_subprocess()



    def subprocess_log_output(self):
        try:  line = self.stdout_queue.get_nowait() # or q.get(timeout=.1)
        except Empty:
            pass # do nothing
        else: # got line
            self.logger.debug(str(line))

        try:  line = self.stderr_queue.get_nowait() # or q.get(timeout=.1)
        except Empty:
            pass # do nothing
        else: # got line
            self.logger.error(str(line))

    def kill_subprocess(self):
        """ kills subprocess started if it was started, otherwise does nothing """
        
        
        # if it exists kill it and clear the blackboard
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception as e:
                print("COULD NOT KILL, {0}".format(self.process))
                raise e
                

        self.blackboard.set(self.alive_key,None)

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

        ON_POSIX = 'posix' in sys.builtin_module_names
        FNULL = open(os.devnull, 'w')
        launch_process = subprocess.Popen(command, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
                close_fds=ON_POSIX,
                bufsize=1,
                universal_newlines=True)
        
        # make sure not to overflow the output, i learned this the hard way
        # subprocess will freeze if you do 
        # want to have a nonblocking way of reading output, start a separate thread for each output channel
        # read continously and store in non blocking to read structure shared with main program

        self.stdout_queue = Queue()
        self.stderr_queue = Queue()

        self.launch_log_thread(launch_process.stdout,self.stdout_queue)
        self.launch_log_thread(launch_process.stderr,self.stderr_queue)

        self.process = launch_process
        self.blackboard.set(self.alive_key,True)
 
    def subprocess_store_output(self,out,queue):
        """ will read the data from the out - io source and put it on the queue, WILL BLOCK the thread """
        while True:
            for line in out:
                queue.put(line)


    def launch_log_thread(self,out,queue):
        """ launches a new thread for storing output for the given io source into the given queue """
        
        thread = Thread(target=self.subprocess_store_output,
            args=(out,queue))
        thread.daemon = True
        thread.start()

        return thread
    def is_subprocess_ok(self):
        """ returns 0 if  subprocess alive, 1 if terminated without error and 2 if terminated with error. Updates state accordingly """


        state = self.process.poll()

        if state is None:
            # ok
            return 0

        elif state == 0:
            # terminated without error
            self.feedback_message = "process exit with return code 0"
            self.process = None
            self.blackboard.set(self.alive_key,None)

            return 1 
            
        else:
            # terminated with error
            (out,err) = self.process.communicate()
            self.feedback_message = "process terminated"
            self.logger.warning("process terminated with sig: {0}".format(-state))
            self.subprocess_log_output()
            self.logger.warning("stderr: {0}".format(err))
            self.logger.warning("stdout: {0}".format(out))

            self.process = None 
            self.blackboard.set(self.alive_key,None)
            return 2

            
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