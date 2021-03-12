import py_trees
import numpy as np
import typing 
import os.path

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
    def __init__(self, name, func : typing.Callable[[],py_trees.common.Status]):

        self.func = func
        super().__init__(name=name)

    def update(self):
        return self.func()

