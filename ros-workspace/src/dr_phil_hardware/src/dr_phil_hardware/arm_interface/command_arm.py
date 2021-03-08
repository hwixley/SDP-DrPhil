
from enum import Enum

import moveit_commander
import sys

class MoveGroup(str,Enum):
    ARM="arm"
    gripper="gripper"


class ArmCommander():
    """ Python API for the moveit interface specific to dr_phil """

    class __ArmCommander():
        def __init__(self) -> None:
            """ 
                Args:
            """
            moveit_commander.roscpp_initialize([])
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_groups = {}
        
            for mg in self.robot.get_group_names():
                    self.move_groups[mg] = moveit_commander.MoveGroupCommander(mg)

        def set_joint_target_async(self,group:MoveGroup,goal:list):
            self.move_groups[group].go(goal,wait=False)

    instance = None

    def __init__(self):
        if not ArmCommander.instance:
            ArmCommander.instance = ArmCommander.__ArmCommander()
        else:
            pass

    def __getattr__(self,name):
        return getattr(self.instance, name)
