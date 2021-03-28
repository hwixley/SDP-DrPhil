import py_trees
import numpy as np
import typing 
import os.path
import datetime 
from dr_phil_hardware.msg import CleaningSchedule,CleaningTime
from py_trees import Status
import time

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

class SetBlackboardVariableCustom(py_trees.behaviour.Behaviour):
    """
    Set the specified variable on the blackboard.

    Args:
        variable_name: name of the variable to set, may be nested, e.g. battery.percentage
        variable_value: value of the variable to set
        overwrite: when False, do not set the variable if it already exists
        name: name of the behaviour
    """
    def __init__(
            self,
            variable_name: str,
            variable_value: typing.Union[typing.Any, typing.Callable[[], typing.Any]],
            overwrite: bool = True,
            name: str=py_trees.common.Name.AUTO_GENERATED,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split('.')
        self.blackboard = py_trees.Blackboard()
        self.variable_value_generator = variable_value if callable(variable_value) else lambda: variable_value
        self.overwrite = overwrite

    def update(self):
        """
        Always return success.

        Returns:
             :data:`~py_trees.common.Status.FAILURE` if no overwrite requested and the variable exists,  :data:`~py_trees.common.Status.SUCCESS` otherwise
        """
        if self.blackboard.set(
            self.variable_name,
            self.variable_value_generator(),
            overwrite=self.overwrite
        ):
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE


class CheckFileExists(py_trees.behaviour.Behaviour):

    def __init__(self, name, file_path):

        self.file_path = file_path
        super().__init__(name=name)

    def update(self):
        if os.path.isfile(self.file_path):
            return py_trees.Status.SUCCESS
        else: 
            return py_trees.Status.FAILURE 

class Lambda(py_trees.behaviour.Behaviour):
    """ Wraps a simple lambda method in a behaviour """
    def __init__(self, name, func : typing.Callable[[],py_trees.common.Status], init : typing.Callable[[],None] = None):

        self.func = func
        self.init = init
        super().__init__(name=name)

    def initialise(self):
        if self.init is not None:
            self.init()

        return super().initialise()
    def update(self):
        return self.func()

class CheckIsOnSchedule(py_trees.Behaviour):

    def __init__(self, name,schedule_src):

        self.schedule_src = schedule_src

        super().__init__(name)

    def in_cleaning_interval(self,time: datetime.datetime,cleaning_time : CleaningTime):
        time_now_seconds = time.hour * 60 * 60 + time.minute * 60
        cleaning_time_on_seconds = cleaning_time.hour_on * 60 * 60 + cleaning_time.minute_on * 60
        cleaning_time_off_seconds = cleaning_time.hour_off * 60 * 60 + cleaning_time.minute_off * 60
        return time_now_seconds >= cleaning_time_on_seconds and time_now_seconds < cleaning_time_off_seconds

    def initialise(self):
        self.feedback_message = ""
        return super().initialise()
    def update(self):
        self.feedback_message="looking for schedule.."
        schedule : CleaningSchedule= py_trees.Blackboard().get(self.schedule_src)
        now = datetime.datetime.now()
        is_weekend = now.weekday() >= 5

        if schedule is None:
            self.feedback_message = "No schedule found"
            return Status.FAILURE
        
        on_schedule_weekday = not is_weekend and self.in_cleaning_interval(now,schedule.weekdays)
        on_schedule_weekend = is_weekend and self.in_cleaning_interval(now,schedule.weekends)
        
    
        start_time = schedule.weekends.hour_on if is_weekend else schedule.weekdays.hour_on
        self.feedback_message = "weekend?:{} , start hour:{}".format(is_weekend, start_time)
        if on_schedule_weekend or on_schedule_weekday:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class WaitForNextClean(py_trees.Behaviour):

    LAST_CLEAN_TARGET="clean/ended"

    def __init__(self, name,schedule_src):

        self.schedule_src = schedule_src

        super().__init__(name)


    def initialise(self):
        py_trees.Blackboard().set(WaitForNextClean.LAST_CLEAN_TARGET,None)
        return super().initialise()

    def update(self):
        last_clean = py_trees.Blackboard().get(WaitForNextClean.LAST_CLEAN_TARGET)

        if last_clean is None:
            last_clean = time.time()
            py_trees.Blackboard().set(WaitForNextClean.LAST_CLEAN_TARGET,last_clean)

        schedule : CleaningSchedule = py_trees.Blackboard().get(self.schedule_src)
        now = datetime.datetime.now()
        is_weekend = now.weekday() >= 5

        if schedule is None:
            return Status.FAILURE
        
        interval = 0
        if is_weekend:
            interval = schedule.weekends.interval
        else:
            interval = schedule.weekdays.interval 

        if time.time() >= interval * 60 + last_clean:
            return Status.SUCCESS

        return Status.RUNNING

class ShouldPreemptAndGoBase(py_trees.Behaviour):
    """Returns SUCCESS, if the robot needs to preempt whatever it's doing (when possible) and head to base
        will be true when return time is a datetime instead of ASAP
    Args:
        py_trees ([type]): [description]
    """
    def __init__(self, name,schedule_src):

        self.schedule_src = schedule_src

        super().__init__(name)
        self.start_ASAP_time = None

    def minutes_till_midnight(self):        
        tomorrow = datetime.date.today() + datetime.timedelta(1)
        midnight = datetime.datetime.combine(tomorrow,datetime.time())
        now = datetime.datetime.now()
        return (midnight - now).seconds / 60

    def update(self):
        schedule : CleaningSchedule = py_trees.Blackboard().get(self.schedule_src)

        if schedule is None:
            return Status.FAILURE


        if schedule.return_time.return_type == "":
            return Status.FAILURE

        datetime_now = datetime.datetime.now()

        datetime_start = None
        if schedule.return_time.return_type == "TIME":
            datetime_start = datetime.datetime(year=schedule.return_time.year,
                                                month=schedule.return_time.month,
                                                day=schedule.return_time.day,
                                                hour=schedule.return_time.hour,
                                                minute=schedule.return_time.minute)
       
        elif schedule.return_time.return_type == "ASAP":
            if self.start_ASAP_time is None:
                self.start_ASAP_time = datetime_start = datetime.datetime.now() - datetime.timedelta(seconds=1)
            else:
                datetime_start = self.start_ASAP_time


        self.feedback_message = "preemption start "
        after_start = datetime_now >= datetime_start
        self.feedback_message = "preemption in: {}".format(datetime_start - datetime_now)

        if after_start:
            # check duration
            delta = None
            if schedule.return_time.return_duration == -1:
                delta = datetime.timedelta(minutes=self.minutes_till_midnight()) + (datetime_now - datetime_start)
            else:
                delta = datetime.timedelta(minutes=int(schedule.return_time.return_duration))

            
            datetime_end = datetime_start + delta 

            self.feedback_message = "preemption started at: {} , lasting: {},  ends at :{}".format(datetime_start,datetime_end - datetime_start,datetime_end)

            if datetime_now < datetime_end:
                return Status.SUCCESS
            else:
                return Status.FAILURE


        else:
            return Status.FAILURE

    def terminate(self, new_status):
        # reset on failure, i.e. when not in preempt zone
        if new_status == Status.FAILURE:
            self.start_ASAP_time = None

        return super().terminate(new_status)