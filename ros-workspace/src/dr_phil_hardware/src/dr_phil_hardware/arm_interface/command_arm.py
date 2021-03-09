
from enum import Enum

import moveit_commander
import sys
import rospy 

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
            mg =  self.move_groups[group]
            rospy.loginfo("setting goal:{} -> {} for move group: {}".format(mg.get_current_joint_values(),goal,group))
            mg.go(goal,wait=False)

    instance = None

    def __init__(self):
        if not ArmCommander.instance:
            ArmCommander.instance = ArmCommander.__ArmCommander()
        else:
            pass

    def __getattribute__(self,name):
        if name == "instance":
            return ArmCommander.instance
        else:
            return getattr(self.instance, name)


    # JUST FOR TYPE HINTS
    def set_joint_target_async(self,group:MoveGroup,goal:list) -> None:
        assert(False)
