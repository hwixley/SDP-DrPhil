import py_trees
import time 

class HoldStateForDuration(py_trees.decorators.Decorator):
    """ Decorator which returns SUCCESS only if child continuously returns the given state without interruption for the given amount of time
    """
    def __init__(self, child, name, duration: int,state : py_trees.common.Status = py_trees.common.Status.SUCCESS, ignore_running : bool = False):
        """
            Parameters:
                child: the decorated child node
                name: the name of this decorator
                duration: the length of time the state needs to be upkept
                state: the state to be continously returned 
                ignore_running: If True will not count running as a different state to the given state 
        """
        super().__init__(child, name=name)

        print(duration)
        self.duration = duration
        self.target_state = state
        self.ignore_running = ignore_running

    def initialise(self):
        self.feedback_message = ""
        self.start_time = time.time()
    
    def update(self):
        time_left = self.duration - (time.time() - self.start_time)

        self.feedback_message = "Holding for: {0}s".format(time_left)
        expired = time_left < 0
        child_status = self.decorated.status

        # while not expired keep checking state
        if not expired:

            in_ignored_status = self.ignore_running and (child_status == py_trees.Status.RUNNING)

            if (child_status != self.target_state) and not in_ignored_status:
                return py_trees.Status.FAILURE
            else:
                return py_trees.Status.RUNNING
        else:
            return py_trees.Status.SUCCESS

    def setup(self, timeout):
        return super().setup(timeout)
        
    def tick(self):
        return super().tick()