
from dr_phil_hardware.behaviour.leafs.ros import ActionClientConnectOnInit
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal
import py_trees
from py_trees.behaviour import Behaviour
import rospy 
import time 
def one_tick_success_leaf_fibonacci(action_ns):
        f = FibonacciGoal()
        f.order = 1
        

        root = ActionClientConnectOnInit("tested",
            action_spec=FibonacciAction,
            action_goal=f,
            action_namespace=action_ns,
            override_feedback_message_on_running="runnning",
            connection_timeout=1)

        root.setup(1)
        return root 

def tick_untill_not_running(root : Behaviour,tick_limit,freq=1,ros_sleep=True):
    rate = None
    if ros_sleep:
        rate = rospy.Rate(freq)


    root.tick_once()

    ticks = tick_limit - 1
    
    while root.status is py_trees.Status.RUNNING and ticks > 0:
        root.tick_once()

        if ros_sleep:
            rate.sleep()
        else:
            time.sleep(1/freq)

        ticks -= 1