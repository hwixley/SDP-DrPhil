
from dr_phil_hardware.behaviour.leafs.ros import ActionClientConnectOnInit
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal

def one_tick_success_leaf(action_ns):
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