

from click import command
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg 

class RobotiqControl:

    def __init__(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    def activate(self):
        self.command.rACT = 1
        self.command.rGTO = 100
        self.command.rSP  = 255
        self.command.rFR  = 150
        return self.command

    def reset(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 0
        return self.command

    def close(self):
        self.command.rPR = 255
        return self.command

    def open(self):
        self.command.rPR = 0
        return self.command

    def increase_speed(self,input):
        increment = input
        self.command.rSP += increment
        if self.command.rSP > 255:
            self.command.rSP = 255
        return self.command

    def decrease_speed(self,input):
        increment = input
        self.command.rSP -= increment
        if self.command.rSP < 0:
            self.command.rSP = 0
        return self.command

    def increase_force(self,input):
        increment = input
        self.command.rFR += increment
        if self.command.rFR > 255:
            self.command.rFR = 255
        return self.command

    def decrease_force(self,input):
        increment = input
        self.command.rFR -= increment
        if self.command.rFR < 0:
           self.command.rFR = 0
        return self.command


