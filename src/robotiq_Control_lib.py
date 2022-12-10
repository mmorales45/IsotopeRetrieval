

from click import command
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg 
"""
Library in python to control the robotiq gripper using the command message type found in their github repo.
"""
class RobotiqControl:
    """
    Class of a robotiq command.
    """
    def __init__(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    def activate(self):
        """
            Function:
                Activate the gripper/turn on.
        """
        self.command.rACT = 1
        self.command.rGTO = 100
        self.command.rSP  = 255
        self.command.rFR  = 150
        return self.command

    def reset(self):
        """
            Function:
                Clear all the command values to 0 and deactivate the gripper.
        """
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 0
        return self.command

    def close(self):
        """
            Function:
                Set the goal position to be closed/max.
        """
        self.command.rPR = 255
        return self.command

    def open(self):
        """
            Function:
                Set the goal position to be open.
        """
        self.command.rPR = 0
        return self.command

    def increase_speed(self,input):
        """
            Function:
                Increase the closing/opening speed.
            args:
                input - The desired speed increment value
        """
        increment = input
        self.command.rSP += increment
        if self.command.rSP > 255:
            self.command.rSP = 255
        return self.command

    def decrease_speed(self,input):
        """
            Function:
                Decrease the closing/opening speed.
            args:
                input - The desired speed increment value
        """
        increment = input
        self.command.rSP -= increment
        if self.command.rSP < 0:
            self.command.rSP = 0
        return self.command

    def increase_force(self,input):
        """
            Function:
                Increase the force of the gripper.
            args:
                input - The desired force increment value.
        """
        increment = input
        self.command.rFR += increment
        if self.command.rFR > 255:
            self.command.rFR = 255
        return self.command

    def decrease_force(self,input):
        """
            Function:
                Decrease the force of the gripper.
            args:
                input - The desired force increment value.
        """
        increment = input
        self.command.rFR -= increment
        if self.command.rFR < 0:
           self.command.rFR = 0
        return self.command


