import threading
from typing import Callable
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
from dr_phil_hardware.arm_interface.command_arm import ArmCommander
import threading
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseArray 
import dynamic_reconfigure
import copy 

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

class JointTargetSetAndForget(py_trees.behaviour.Behaviour):
    """ behaviour which sets move groups target and leaves it running with SUCCESS """

    def __init__(self,name,move_group,joint_target):
        
        super().__init__(name=name)
        self.target = joint_target 
        self.move_group = move_group


    def initialise(self):
        pass

    def update(self):
        commander = ArmCommander()
        commander.set_joint_target_async(self.move_group,self.target)
        return py_trees.Status.SUCCESS
        
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
                action_goal (:obj:`any`): preconfigured action goal (e.g. move_base_msgs.msg.MoveBaseGoal()) or callable returning goal
                action_namespace (:obj:`str`): where you can find the action topics
                override_feedback_message_on_running (:obj:`str`): override the feedback message from the server
                connection_timeout (:obj:`float`) : the time to wait for a connection on initialisation, after which the behaviour will fail

            Feedback messages are often accompanied with detailed messages that continuously change - since these
            are not significant and we don't want to log every change due to these messages, you can provide an override
            here that simply signifies the action is running.

        """
        self.action_maybe_callable = action_goal
        self.action_goal = None
        super().__init__(name=name, action_spec=action_spec, action_goal=None, action_namespace=action_namespace, override_feedback_message_on_running=override_feedback_message_on_running)
        
        self.connection_timeout = connection_timeout
    def setup(self, timeout):
        # we override the default setup behaviour which happens 
        # when the tree is initialized
        return True 

    def initialise(self):

        super().initialise()

        # take goal from blackboard possibly
        self.action_goal = self.action_maybe_callable() if callable(self.action_maybe_callable) else self.action_maybe_callable
        
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




class CreateMoveitTrajectoryPlan(py_trees.Behaviour):
    """ creates a plan for the given move group to follow the trajectory given on the blackboard under "moveit/pose_targets" as (:obj:`PoseArray`) message, fails if fraction is below given threshold.
        saves plan on blackboard at "moveit/plan" 
    """
    WAYPOINT_SOURCE=  "moveit/pose_targets"
    PLAN_TARGET="moveit/plan"

    def __init__(self, name,move_group : MoveGroup,fraction_threshold = 1,pose_frame="base_footprint",pose_target_include_only_idxs=None):
        """
            Args:
                fraction_threshold: the fraction of the trajectory needing to be achieved for the plan to be considered successfull,
                    if set to 0, any plan will be valid
                pose_target_include_only_idxs: if set, will include only those indexes from pose targets specified

        """
        super().__init__(name)
        self.fraction_threshold = fraction_threshold
        self.blackboard = py_trees.Blackboard()
        self.move_group = move_group
        self.pose_frame = pose_frame
        self.include_idxs = pose_target_include_only_idxs

        # resetable
        self.fraction = None
        self.trajectory = None
        self.thread = None
        self.planning_complete = False
        self.waypoints = None 

    def initialise(self):
        self.fraction = None
        self.trajectory = None
        self.planning_complete = False
        self.thread = None
        self.waypoints = None

        super().initialise()



    def update(self):
        # if planning hasn't started
        if not self.thread:
            # get waypoints 
            self.waypoints : PoseArray = copy.deepcopy(self.blackboard.get(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE))
            
            if not self.waypoints:
                    self.feedback_message = "No waypoints at {}".format(CreateMoveitTrajectoryPlan.WAYPOINT_SOURCE)
                    return py_trees.Status.FAILURE
            elif len(self.waypoints.poses) <= 0:
                    self.feedback_message = "Empty waypoints list"
                    return py_trees.Status.FAILURE
            elif isinstance(self.waypoints,PoseArray):

                if self.include_idxs:
                    waypoints_new_poses = [self.waypoints.poses[x] for x in self.include_idxs if len(self.waypoints.poses) -1 >= x]
                    self.waypoints.poses = waypoints_new_poses
            
                # start planning
                self.thread = threading.Thread(target= self.__blocking_plan,args = (self.waypoints,))
                self.thread.start()
                return py_trees.Status.RUNNING
            else:
                self.feedback_message = "Waypoints is not of the right type"
                return py_trees.Status.FAILURE

        # running planning
        elif self.thread.is_alive():

            self.feedback_message = "Planning.."
            return py_trees.Status.RUNNING

        # planning finished
        else:
            # check it's valid
            if self.fraction < self.fraction_threshold:
                self.feedback_message = "Fraction too low"
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "Found valid plan"
                # success
                self.blackboard.set(CreateMoveitTrajectoryPlan.PLAN_TARGET,self.trajectory)
                return py_trees.Status.SUCCESS


    def terminate(self, new_status):
        # "drop" the planning thread if running
        self.thread = None

        super().terminate(new_status)

    def __blocking_plan(self,waypoints : PoseArray):
        """ starts planning, and blocks thread """
        #TODO: check for race conditions on terminate, and anotehr startup swiftly after
        assert(isinstance(waypoints,PoseArray))
        ac = ArmCommander()
        ac.clear_state(self.move_group)
        ac.set_pose_planning_frame(self.move_group,self.pose_frame)

        (self.trajectory,self.fraction) = ac.plan_smooth_path(self.move_group,[x for x in waypoints.poses])
        sys.exit()

class ExecuteMoveItPlan(py_trees.Behaviour):
    """ Executes plan at moveit/plan on the blackboard, returns true once succeeded, fails if some point is not reached
    """
    PLAN_SOURCE=CreateMoveitTrajectoryPlan.PLAN_TARGET

    def __init__(self, name,move_group : MoveGroup, distance_waypoint_threshold = 0.02,time_threshold_mult=0.1):
        """[summary]

        Args:
            name (:obj:`str`): the name of the behaviour
            move_group (:obj:`MoveGroup`): the move group to use
            distance_waypoint_threshold (:obj:`float`, optional): the distance the end effector can deviate from the path at any waypoint at the required time. Defaults to 0.01.
            time_threshold_mult: the leeway added to distance_waypoint_threshold for every second in polling delay
        """
        
        super().__init__(name)
        self.started : bool = False 
        self.curr_point_idx : int = None
        self.plan = None 

        self.move_group = move_group
        self.distance_waypoint_threshold = distance_waypoint_threshold
        self.time_threshold_mult = time_threshold_mult

    def initialise(self):

        self.started = False 
        self.curr_point_idx = 0
        self.plan : RobotTrajectory = py_trees.Blackboard().get(ExecuteMoveItPlan.PLAN_SOURCE)

    def update(self):
        if self.started:

            curr_point = self.plan.joint_trajectory.points[self.curr_point_idx]
            target_time = self.plan.joint_trajectory.header.stamp + curr_point.time_from_start
            time_late = rospy.Time.now() - target_time

            # keep going through points only if we are either on time, or late, 
            # always work from the most "recent point" since last tick
            # i.e. curr_point_idx is always the point which is not in the past
            while time_late.to_sec() >= 0:
                self.curr_point_idx += 1

                # if went through last point, succeed
                if self.curr_point_idx + 1 > len(self.plan.joint_trajectory.points):
                    self.feedback_message = "Executed plan"
                    return py_trees.Status.SUCCESS

                curr_point = self.plan.joint_trajectory.points[self.curr_point_idx]
                target_time = self.plan.joint_trajectory.header.stamp + curr_point.time_from_start
                time_late = rospy.Time.now() - target_time
            
            # we work out if the last pose aligns with the expected pose
            # we figure out the maximum error reasonable relative to our poll rate
            # using jacobian matrix we figure out the time the joint could have moved while we 
            # "weren't looking"
            # samples: X---X---X---X---X
            #    traj: X-X---X---X---X--
            # TODO: make this work, or use action server directly
            # last_point = self.plan.joint_trajectory.points[self.curr_point_idx - 1] if self.curr_point_idx >= 1 else None
            
            # if last_point:
            #     current_pos_ee = ArmCommander().get_current_pose(self.move_group).pose.position
            #     current_pos_ee = np.array([current_pos_ee.x,current_pos_ee.y,current_pos_ee.z])

            #     # assume linear velocity at the ee
            #     J = ArmCommander().get_jacobian(self.move_group,list(last_point.positions))
            #     last_velocity_ee = (J @ np.array(last_point.velocities)[:,np.newaxis])[0:3,:]
                
            #     # figure out "perfect state" at last point
            #     last_state = ArmCommander().get_current_robot_state()
            #     last_state.joint_state.name = ArmCommander().get_joints(self.move_group)
            #     last_state.joint_state.position = last_point.positions
            #     last_state.joint_state.velocity = last_point.velocities
            #     last_state.joint_state.effort = last_point.effort

            #     # figure out required ee pos last point
            #     (poses,_,_) = ArmCommander().compute_fk(
            #         [ArmCommander().get_end_effector_link(self.move_group)],
            #         ArmCommander().get_planning_frame(self.move_group),
            #         last_state)
                
            #     last_ee_pos = np.array([poses[0].pose.position.x,
            #         poses[0].pose.position.y,
            #         poses[0].pose.position.z])

            #     secs_since_last_ee = (rospy.Time.now() - (last_point.time_from_start + self.plan.joint_trajectory.header.stamp)).to_sec()

            #     # error at interpolated point from perfect target pose
            #     error = np.linalg.norm((last_ee_pos + (last_velocity_ee * secs_since_last_ee)) - current_pos_ee)
            #   

            #     if  error > self.distance_waypoint_threshold + self.time_threshold_mult * secs_since_last_ee:
            #         self.feedback_message = "Error exceeded threshold, failing trajectory"
            #         return py_trees.Status.FAILURE


            return py_trees.Status.RUNNING
        else:
            if self.plan is None:
                self.feedback_message = "Plan at {} is None".format(ExecuteMoveItPlan.PLAN_SOURCE)
                return py_trees.Status.FAILURE
            elif len(self.plan.joint_trajectory.points) == 0:
                self.feedback_message = "Empty plan, succeed"
                return py_trees.Status.SUCCESS

            self.started=True
            ac = ArmCommander()
            # start off the plan
            self.feedback_message = "Executing plan..."
            ac.execute_plan(self.move_group,self.plan,blocking=False)
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # TODO: halt arm commander
        return super().terminate(new_status)


class CallService(py_trees.Behaviour):
    """ calls a service and returns success if call was successfull """
    def __init__(self, name,service_topic,service_type,service_content,blackboard_target,pre_process_callable : Callable[[object],object]=None, callable_none_is_failure=False):
        """[summary]

        Args:
            name ([type]): [description]
            service_topic ([type]): [description]
            service_type ([type]): [description]
            service_content ([type]): [description]
            blackboard_target ([type]): [description]
            pre_process_callable ([type], optional): [description]. Defaults to None. optional function to process the response before putting it on the blackboard, returning None here will be interpreted as Failure if none is failure is set. exceptions are failures by default
        """
        self.service_topic = service_topic
        self.service_type = service_type    
        self.service_content = service_content
        self.blackboard_target = blackboard_target 
        self.pre_process_callable = pre_process_callable
        self.callable_none_is_failure = callable_none_is_failure
        self.proxy = rospy.ServiceProxy(self.service_topic,self.service_type)
        self.return_value = None
        self.failed = False
        self.thread = None
        self.processing = False
        self.exception = None
        super().__init__(name=name)

    def initialise(self):
        self.return_value = None 
        self.failed = False
        self.thread = None 
        self.processing = False 
        self.exception = None
        self.fail_exception = None

        return super().initialise()

    def __blocking_call(self,argument): 
        # TODO: Race conditions on terminate on preempt followed by new initialize
        try:
            self.return_value = self.proxy.call(argument)
        except Exception as E:
            self.failed = True
            self.exception = E
            rospy.logfatal("ASDSDADSA\n|ASDASDASDSDA")
            sys.exit(1)
        sys.exit(0)

    def update(self):
        self.feedback_message = "Calling proxy.."

        if not self.processing and self.return_value is None:

            self.thread = threading.Thread(target=self.__blocking_call,args=(self.service_content,))
            
            self.thread.start()
            self.processing = True 
            return py_trees.Status.RUNNING

        elif self.processing and self.return_value is None and self.failed == False:
            return py_trees.Status.RUNNING
        elif self.failed == True:
            self.feedback_message = "Failed: {}".format(self.exception)
            return py_trees.Status.FAILURE
        else:

            value = self.return_value
            if callable(self.pre_process_callable):
                try:
                    value = self.pre_process_callable(value)
                except Exception as E:
                    self.feedback_message = "{}".format(E)
                    return py_trees.Status.FAILURE
     

            if value is None and self.callable_none_is_failure:
                return py_trees.Status.FAILURE
            py_trees.Blackboard().set(self.blackboard_target,self.return_value)

            return py_trees.Status.SUCCESS

class DynamicReconfigure(py_trees.Behaviour):

    def __init__(self, name,topic,update_dict):

        self.topic = topic
        self.update_dict = update_dict
        super().__init__(name=name)


    def callback(self,config):
        self.feedback_message = str(config)
    

    def initialise(self):

        return super().initialise()
    
    def update(self):
        self.client = dynamic_reconfigure.client.Client(self.topic,timeout=10,config_callback=self.callback)
        self.feedback_message = "updating configuration.."
        if self.client is not None:
            self.client.update_configuration(self.update_dict)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "No client available"
            return py_trees.Status.FAILURE

