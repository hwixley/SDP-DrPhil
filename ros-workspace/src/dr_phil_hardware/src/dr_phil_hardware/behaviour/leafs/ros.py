import threading
from numpy.core.fromnumeric import std
import py_trees
from py_trees.common import Status
import py_trees_ros
import rospy
import numpy as np
import subprocess
import os 
import signal
from threading  import Thread
import sys
import time 
import actionlib

try:
    from queue import Queue,LifoQueue, Empty
except ImportError:
    from Queue import Queue,LifoQueue, Empty  # python 2.x

from dr_phil_hardware.arm_interface.command_arm import ArmCommander,MoveGroup

log_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..","..","logs")

class RunRos(py_trees.behaviour.Behaviour):
    """ Runs and returns RUNNING status for as long as given process is running, will return FAILURE whenever it halts, and if killed via blackboard kill key will return SUCCESS and
        won't reset untill this key is cleared (i.e. will keep returning SUCCESS ). Optionally can serve as short process launcher
    """
    def __init__(self,name,blackboard_alive_key,blackboard_kill_trigger_key,launch_file=None,node_file=None,package="dr_phil",args=[],success_on_non_error_exit=False):
        """
            Args: 
                name: name of the behaviour
                blackboard_alive_key: the key which will be set as long as the process is running
                blackboard_kill_trigger_key: the key which if set will cause the node to kill the process and keep returning SUCCESS as long as it's set (on the next tick of this node)
                launch_file: the full filename of the launchfile
                package: the package containing the launch file (DEFAULT: "dr_phil")
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

            # if process is still running successfully
            if ok == 0:
                # check kill trigger
                if self.blackboard.get(self.kill_key) is not None:
                    # kill
                    self.logger.debug("process ended stdout dump: "+self.get_last_n_lines(self.stdout_queue,50))
                    self.kill_subprocess()
                    self.last_success = True

                    return py_trees.Status.SUCCESS

                return py_trees.Status.RUNNING

            # if process died when not expected
            elif ok == 2 or (ok == 1 and not self.success_on_non_error_exit):

                self.logger.error("stderr dump: "+self.get_last_n_lines(self.stderr_queue,50))
                self.logger.error("stdout dump: "+self.get_last_n_lines(self.stdout_queue,50))
                return py_trees.Status.FAILURE

            # if process died but we expected it
            else:
                self.logger.debug("process ended stdout dump: "+self.get_last_n_lines(self.stdout_queue,50))
                self.last_success = True
                return py_trees.Status.SUCCESS

        else:
            # start a subprocess
            self.start_subprocess()


            return py_trees.Status.RUNNING

    def terminate(self,newState):

        # before we enter each initialize or after we are pre-empted
        # we make sure to not leave hanging processes
        self.kill_subprocess()



    def get_last_n_lines(self,queue,n):
        if queue is None:
            return

        l = ""
        for i in range(n):
            try:  line = self.stdout_queue.get_nowait() # or q.get(timeout=.1)
            except Empty:
                pass # do nothing
            else: # got line
                l += "\n" + str(line)
        return l

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
        
        self.feedback_message = command

        ON_POSIX = 'posix' in sys.builtin_module_names

        # log = open(os.path.join(log_dir,"{0}.log".format(self.name)),"w")
        # log.truncate(0)

        launch_process = subprocess.Popen(command, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
                close_fds=ON_POSIX,
                # bufsize=1,
                # universal_newlines=True
        )
        # make sure not to overflow the output, i learned this the hard way
        # subprocess will freeze if you do 
        # want to have a nonblocking way of reading output, start a separate thread for each output channel
        # read continously and store in non blocking to read structure shared with main program

        self.stdout_queue = LifoQueue()
        self.stderr_queue = LifoQueue()

        self.launch_log_thread(launch_process.stdout,self.stdout_queue)
        self.launch_log_thread(launch_process.stderr,self.stderr_queue)

        self.process = launch_process
        self.blackboard.set(self.alive_key,True)
 
    def subprocess_store_output(self,out,queue):
        """ will read the data from the out - io source and put it on the queue, WILL BLOCK the thread """
        while True:
            try:
                for line in out:
                    queue.put(line)
            except:
                #  locked, just wait
                print("out is locked")
                time.sleep(5)
                pass

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
            self.logger.warning("stderr: {0}".format(err))
            self.logger.warning("stdout: {0}".format(out))

            self.process = None 
            self.blackboard.set(self.alive_key,None)
            return 2

            
class PublishTopic(py_trees.behaviour.Behaviour):
    """ a behaviour which publishes a certain message and returning RUNNING on success. Can be set to return success on publish"""

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

# class MoveGroupJointTarget(py_trees.behaviour.Behaviour):
#     """ a behaviour which publishes a certain message and returning RUNNING on success. Can be set to return success on publish"""

#     def __init__(self,):
#         """ 
#         Args:
#             name: the name of the behaviour
#             msg_type: the type of message to be published
#             topic: the topic on which to publish the message
#             queue_size: the publisher queue size
#             success_on_publish: when true, will return SUCCESS after publishing each time
#             success_after_n_publishes: when set to any integer, will return success after publishing n times without failure
#         """

#         super().__init__(name=name)
#         self.msg = msg


#         self.publisher = rospy.Publisher(topic,msg_type,queue_size=queue_size)
#         self.success_on_publish = success_on_publish    
#         self.n_target = -1 if success_after_n_publishes is None else success_after_n_publishes

#     def initialise(self):
#         pass

#     def update(self):
#         self.feedback_message = "Waiting for data"
#         try:
#             self.feedback_message = "Published"
#             self.publisher.publish(self.msg)
#         except:
#             self.feedback_message = "Publisher failure"
#             return py_trees.common.Status.FAILURE

#         if self.success_on_publish or self.n_target >= 0:
#             self.n_target -= 1
#             if self.n_target > 0:
#                 return py_trees.common.Status.RUNNING
#             else:
#                 return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.RUNNING       



class MessageChanged(py_trees_ros.subscribers.Handler):
    
    """ Listens to topic and returns running before receiving 2 messages, which are then compared for changes
        If they have changed SUCCESS is returned, and otherwise FAILURE
    """

    def __init__(self, name, topic_name, topic_type,compare_method=None,waiting_timeout=None,timeout_status : py_trees.Status = py_trees.Status.FAILURE):
        """ 
            Args:
                name: the name of the behaviour
                topic_name: the topic to listen to for changes
                topic_type: the type of message to expect,
                compare_method: a function which takes in (msg1,msg2) of topic_type and returns bool (true => changed)
                waiting_timeout: the time after which if no second message is received, timeout is triggered
                timeout_status: the status to return on timeout
        """

        super().__init__(name=name, topic_name=topic_name, topic_type=topic_type)
        self.second_msg = None
        self.first_msg = None
        self.waiting_timeout = waiting_timeout
        self.timeout_status = timeout_status
        self.compare_method = compare_method

    def initialise(self):
        
        self.start_time = time.time()
        self.first_msg = None
        self.second_msg = None 


        return super().initialise()


    def message_changed(self,msg1,msg2):
        if self.compare_method is not None:
            return self.compare_method(msg1,msg2) 
        else:
            return not msg1 == msg2

    def update(self):
        
        # we always wait for first message with RUNNING
        with self.data_guard:
            if self.first_msg == None:
                self.feedback_message = "waiting for first message"
                if self.msg != None:
                    self.first_msg = self.msg
                    self.msg = None
                    self.feedback_message = "waiting for second message"
                    return py_trees.Status.RUNNING
                else:
                    return py_trees.Status.RUNNING

            # then we possibly timeout on waiting for second

            timeout = self.waiting_timeout is not None 

            if timeout: 
                time_left = (time.time() - self.start_time) - self.waiting_timeout

                if time_left <= 0:
                    self.feedback_message = "timed out"
                    return self.timeout_status

            # await second
            if self.second_msg == None:
                if self.msg != None:
                    self.second_msg = self.msg
                    self.msg = None
                else:
                    return py_trees.Status.RUNNING
                
            # do comparison, only reached with first and second message present
            # and if not timed out

            if self.message_changed(self.first_msg,self.second_msg):
                self.feedback_message = "messages changed"
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = "messages the same"
                return py_trees.Status.FAILURE



class ActionClientConnectOnInit(py_trees_ros.actions.ActionClient):
    """ A version of the action client behaviour which initializes the 
        connection to the server in init """

    def __init__(self, name, action_spec, action_goal, action_namespace, override_feedback_message_on_running, connection_timeout=1):
        """ 
            A generic action client interface. This simply sends a pre-configured
            goal to the action client.

            Cases where you might want to subclass and extend this behaviour:

            * Update the goal data in :meth:`~py_trees_ros.actions.ActionClient.initialise()`

            * e.g. use blackboard data to determine the new characteristics of your goal
            * Trigger true pre-emption by sending a new goal in :meth:`~py_trees_ros.actions.ActionClient.update()`

            Args:
                name (:obj:`str`): name of the behaviour
                action_spec (:obj:`any`): spec type for the action (e.g. move_base_msgs.msg.MoveBaseAction)
                action_goal (:obj:`any`): preconfigured action goal (e.g. move_base_msgs.msg.MoveBaseGoal())
                action_namespace (:obj:`str`): where you can find the action topics
                override_feedback_message_on_running (:obj:`str`): override the feedback message from the server
                connection_timeout (:obj:`float`) : the time to wait for a connection on initialisation, after which the behaviour will fail

            Feedback messages are often accompanied with detailed messages that continuously change - since these
            are not significant and we don't want to log every change due to these messages, you can provide an override
            here that simply signifies the action is running.

        """
        super().__init__(name=name, action_spec=action_spec, action_goal=action_goal, action_namespace=action_namespace, override_feedback_message_on_running=override_feedback_message_on_running)

        self.connection_timeout = connection_timeout

    def setup(self, timeout):
        # we override the default setup behaviour which happens 
        # when the tree is initialized
        return True 

    def initialise(self):

        super().initialise()

        self.action_client = None
        self.connected = False
        self.time_start = rospy.get_time()

    def update(self):
       # we change slightly the behaviour on when action client is missing
       # and also setup the action client here in a non blocking way
        time_left = self.connection_timeout - (rospy.get_time() -  self.time_start)
        if not self.connected and time_left > 0:
            if not self.action_client:
                self.action_client = actionlib.SimpleActionClient(
                    self.action_namespace,
                    self.action_spec
                )
            if self.action_client.wait_for_server(
                rospy.Duration(min(time_left,1))): # wait for at most a second each time
                self.connected = True 
            else: return py_trees.Status.RUNNING             

        # the super() implementation will expect self.action_client to be non null iff connected
        if not self.connected:
            self.action_client = None 
            self.logger.error("{0}.could not connect to the action server at '{1}'".format(self.__class__.__name__, self.action_namespace))

        super_state = super().update()

        # we just don't return invalid as super does, flip it to failure
        if super_state == py_trees.Status.INVALID:
            return py_trees.Status.FAILURE
        else:
            return super_state