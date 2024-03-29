#!/usr/bin/env python
"""
This node is a GUI that can be used to make it easier for a user to do basic control of the system such as turning on/off, running scripts and doing basic motion control 
through a GUI and not having to use the command line for most things.

"""
from multiprocessing import cpu_count
import sys
from PyQt5 import QtGui
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QToolTip, QMessageBox, QLabel)
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QGridLayout, QWidget, QMenu,QDesktopWidget
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize, QTimer

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


import rospy    
import sys
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QGridLayout, QWidget, QMenu,QDesktopWidget
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize, QTimer

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import os
import signal
import subprocess
import roslaunch
from argonne_final_project.LaunchRunClass import *
from moveit_commander.conversions import pose_to_list, list_to_pose

from geometry_msgs.msg import Pose


import time
import sys
path = sys.path[0]

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
from std_srvs.srv import Empty, Trigger
from argonne_final_project.srv import CPose, Quat


import atexit
print('is this first')
def exit_handler():
	kill_all()
atexit.register(exit_handler)



        
list_of_classes = []
def kill_all():
    """
        Function:
            Kill all instances of a node.
    """

    print("Stoping all Processes...")
    for class_name in list_of_classes:
        class_name.stop()
    print("All Processes Stopped")


powerOnur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POn_UR5e")
list_of_classes.append(powerOnur5e)

def powerOn5e():
    """
        Function:
            Turn on the UR5e, do not release the brakes.
    """
    print("Powering On UR5e")
    powerOnur5e.start()

powerOnur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POn_UR16e")
list_of_classes.append(powerOnur5e)

def powerOn16e():
    """
        Function:
            Turn on the UR16e, do not release the brakes.
    """
    print("Powering On UR16e")
    powerOnur16e.start()
##
powerOffur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POff_UR5e")
list_of_classes.append(powerOffur5e)

def powerOff5e():
    """
        Function:
            Turn off the UR5e.
    """
    print("Powering Off UR16e")
    powerOffur5e.start()

powerOffur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POff_UR16e")
list_of_classes.append(powerOffur16e)

def powerOff16e():
    """
        Function:
            Turn off the UR16e.
    """
    print("Powering Off UR16e")
    powerOffur16e.start()
##
BRur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-BR_UR5e")
list_of_classes.append(BRur5e)

def br5e():
    """
        Function:
            Turn on the UR5e and release the brakes.
    """
    print("Powering Off UR16e")
    BRur5e.start()

BRur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-BR_UR16e")
list_of_classes.append(BRur16e)     

def br16e():
    """
        Function:
            Turn on the UR16e and release the brakes.
    """
    print("Powering Off UR16e")
    BRur16e.start()


class Window(QMainWindow):
    def __init__(self):
        """
        Create the main interface of the GUI.
        """
        super(Window,self).__init__()
        self.setStyleSheet("background-color: white;")
        self.title = "GUI"
        self.top = 100
        self.left = 100
        self.width = 680
        self.height = 500

        self.pushButton = QPushButton("Hardware Menu", self)
        self.pushButton.resize(200, 50  )
        self.pushButton.move(225, 100)
        self.pushButton.setToolTip("<h3>Start the Session</h3>")

        self.pushButton2 = QPushButton("EE Movement", self)
        self.pushButton2.resize(200, 50  )
        self.pushButton2.move(225, 200)
        self.pushButton2.setToolTip("<h3>Start the Session</h3>")

        self.pushButton3 = QPushButton("Joint Control", self)
        self.pushButton3.resize(200, 50  )
        self.pushButton3.move(225, 300)
        self.pushButton3.setToolTip("<h3>Start the Session</h3>")

        self.pushButton4 = QPushButton("Actions", self)
        self.pushButton4.resize(200, 50  )
        self.pushButton4.move(225, 400)
        self.pushButton4.setToolTip("<h3>Start the Session</h3>")

        self.pushButton.clicked.connect(self.window2)              # <===
        self.pushButton2.clicked.connect(self.window3)              # <===
        self.pushButton3.clicked.connect(self.window4)              # <===
        self.pushButton4.clicked.connect(self.window5)              # <===


        self.main_window()

    def main_window(self):
        """
            Function:
                Create the main window of the main menu.
        """
        self.label = QLabel("Manager", self)
        self.label.move(285, 175)
        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)
        self.show()

    def window2(self):                                             # <===
        """
            Function:
                Go to the main menu window, useful when on other menus.
        """
        self.w = CreateGUi()
        self.w.show()
        self.hide()
    def window3(self):                                             # <===
        """
            Function:
                Go to the Menu that allows for pose input and control.
        """
        self.w = Ui_MainWindow()
        self.w.show()
        self.hide()

    def window4(self):                                             # <===
        """
            Function:
                Go to the joint control menu.
        """
        self.w = Jsp_MainWindow()
        self.w.show()
        self.hide()
    
    def window5(self):                                             # <===
        """
            Function:
                Go to the menu for hardware functions such as turning on and off the robot.
        """
        self.w = ActionsCLass()
        self.w.show()
        self.hide()
    

class CreateGUi(QMainWindow):
    """
        Create the Gui is in charge of control robot functions.
    """
    def __init__(self):
        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(700, 350))    
        self.setWindowTitle("Mobile Dual UR System") 
        
        all_buttons = self.createAllButtons()
        
        # for button in all_buttons:
        #     button.clicked.connect()
        self.bt1.clicked.connect(self.Button1)
        self.bt2.clicked.connect(self.Button2)
        self.bt3.clicked.connect(self.Button3)
        self.bt4.clicked.connect(self.Button4)
        self.bt5.clicked.connect(self.Button5)
        self.bt6.clicked.connect(self.Button6)
        self.bt7.clicked.connect(self.Button7)
        self.bt8.clicked.connect(self.Button8)
        self.bt9.clicked.connect(self.ButtonReturn)
        self.bt10.clicked.connect(self.Button10)
        self.bt11.clicked.connect(self.Button11)
        self.bt13.clicked.connect(self.Button13)
        self.bt14.clicked.connect(self.Button14)
        self.bt15.clicked.connect(self.Button15)
        self.bt16.clicked.connect(self.Button16)
        self.bt17.clicked.connect(self.Button17)
        self.bt18.clicked.connect(self.Button18)
        self.bt19.clicked.connect(self.Button19)
        self.bt20.clicked.connect(self.Button20)
        # self.bt10.clicked.connect(self.Button10)
        # self.bt11.clicked.connect(self.Button11)
        
        self.center()
        self.show()
        
    def Button1(self):
        """
            Function:
                Power on the UR5e
        """
        powerOn5e()
        
    def Button2(self):
        """
            Function:
                Power on the UR16e
        """
        powerOn16e()

    def Button3(self):
        """
            Function:
                Power off the UR5e
        """
        powerOff5e()
        
    def Button4(self):
        """
            Function:
                Power off the UR16e
        """
        powerOff16e()
    
    def Button5(self):
        """
            Function:
                Power on the UR5e and release the brakes
        """
        br5e()
        
    def Button6(self):
        """
            Function:
                Power on the UR16e and release the brakes
        """
        br16e()

    def Button7(self):
        """
            Function:
                Play the last script that was loaded on the UR5e, should be the External Control script.
        """
        ur5e_play_srv()
        
    def Button8(self):
        """
            Function:
                Play the last script that was loaded on the UR16e, should be the External Control script.
        """
        ur16e_play_srv()

    def Button10(self):
        """
            Function:
                Stop the script from playing. Should be able to stop ROS control.
        """
        ur5e_stop_srv()
        
    def Button11(self):
        """
            Function:
                Stop the script from playing for the UR16e. Should be able to stop ROS control.
        """
        ur16e_stop_srv()

    def Button13(self):
        """
            Function:
                Unlock the UR5e from the protective stop, usually after collision.
        """
        ur5e_proRelease_srv()
        
    def Button14(self):
        """
            Function:
                Close the popup that can appear on the UR16e's computer, such as the positin verified issue.
        """
        ur16e_closePOP_srv()
    
    def Button15(self):
        """
            Function:
                Close the popup that can appear on the UR5e's computer, such as the positin verified issue.
        """
        ur5e_closePOP_srv()
        
    def Button16(self):
        """
            Function:
                Unlock the UR16e from the protective stop, usually after collision.
        """
        ur16e_proRelease_srv()

    def Button17(self):
        """
            Function:
                Make the System put the object in front of the mir base.
        """
        analyze_object_srv()

    def Button18(self):
        """
            Function:
                Make the System pick up the goal object using the UR5e
        """
        retrieve_object_srv()

    def Button19(self):
        """
            Function:
                Assemble the object using the UR16e.
        """
        assememble_object_srv()

    def Button20(self):
        """
            Function:
                Using the UR16e, disassemble the object.
        """
        disassemble_object_srv()

    def ButtonReturn(self):
        """
            Function:
                Return to the main window, the one that is the parent of the current one.
        """
        self.returnWindow()

    def Refresh(self):
        """
            Function:
                Refresh the window to update it based on the button pressed.
        """
        if self.count > 0:
            self.bt3.setText(str(self.count)+' seconds')
            self.count -= 1
        else:
            self.time.stop()
            self.bt3.setEnabled(True)
            self.bt3.setText('Button 3')
            self.count = 10
    def center(self):
        """
            Function:
                Center the window.
        """
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def createSingleButton(self,title,length,width,color):
        """
            Function:
                Function used to create a button of a color type.
            args:
                title - Title of the button
                length - The length of the button
                width - The width of the button
                color - The color of the button
        """
        button = QPushButton(title,self)
        button.resize(length, width)
        if color == "green":
            button.setStyleSheet("background-color : green")
        elif color == "red":
            button.setStyleSheet("background-color : red")
        elif color == "blue":
            button.setStyleSheet("background-color : blue")
        elif color == "orange":
            button.setStyleSheet("background-color : orange")

        return button
    def space_out(self,list,space,x,y):
        """
            Function:
                Space out the list of buttons to make it uniformed and not clustered.
            args:
                list - The list of buttons 
                space - space between the buttons
                x - the x position of the first button on the list
                y - the y position of the first button on the list
        """
        current_x = x
        for i in list:
            i.move(current_x,y)
            current_x += space
            
    def createAllButtons(self):
        """
            Function:
                Create all the buttons on the current page of the GUI.
        """
        length = 140
        width = 45
        self.bt1 = self.createSingleButton("PowerOn UR5e",length,width,"green")
        self.bt2 = self.createSingleButton("PowerOn UR16e",length,width,"green")

        self.bt3 = self.createSingleButton("PowerOff UR5e",140,width,"red")
        self.bt4 = self.createSingleButton("PowerOff UR16e",140,width,"red")
        self.bt5 = self.createSingleButton("RelBrake UR5e",140,width,"orange")
        self.bt6 = self.createSingleButton("RelBrake UR16e",140,width,"orange")

        self.bt7 = self.createSingleButton("Load Script UR5e ",140,width,"blue")
        self.bt8 = self.createSingleButton("Load Script UR16e",140,width,"blue")
        self.bt9 = self.createSingleButton("Return To Main",140,width,None)

        self.bt10 = self.createSingleButton("Stop Script UR5e",140,width,"red")
        self.bt11 = self.createSingleButton("Stop Script UR16e",140,width,"red")
        self.bt12 = self.createSingleButton("Test",140,width,None)

        self.bt13 = self.createSingleButton("Unlock UR5e STOP",140,width,"blue")
        self.bt14 = self.createSingleButton("Unlock UR16e STOP",140,width,"blue")
        self.bt15 = self.createSingleButton("Close 5e Popup",140,width,"blue")
        self.bt16 = self.createSingleButton("Close 16e Popup",140,width,"blue")

        self.bt17 = self.createSingleButton("Analyze",140,width,"blue")
        self.bt18 = self.createSingleButton("Retrieve",140,width,"blue")
        self.bt19 = self.createSingleButton("Assemble",140,width,"blue")
        self.bt20 = self.createSingleButton("Disassemble",140,width,"blue")

        row1 =  [self.bt1, self.bt2]
        row2 = [self.bt3, self.bt4, self.bt5, self.bt6]
        row3 = [ self.bt7, self.bt8, self.bt9]
        row4  = [ self.bt10, self.bt11, self.bt12]
        row5  = [ self.bt13, self.bt14, self.bt15,self.bt16]
        row6  = [ self.bt17, self.bt18, self.bt19,self.bt20]

        all_buttons = row1 + row2
        self.space_out(row1,space=150,x=200,y=50)
        #Second Row
        self.space_out(row2,space=150,x=50,y=100)

        #Third Row
        self.space_out(row3,space=150,x=50,y=150)
        self.space_out(row4,space=150,x=50,y=200)
        self.space_out(row5,space=150,x=50,y=250)
        self.space_out(row6,space=150,x=50,y=300)
        return all_buttons
    
    def returnWindow(self):                                             # <===
        print('hi')
        self.w = Window()
        self.w.show()
        self.hide()

class ActionsCLass(QMainWindow):
    def __init__(self):
        """
        Create a menu of actions, such as assembly and position control.
        """
        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(700, 350))    
        self.setWindowTitle("Mobile Dual UR System") 
        
        all_buttons = self.createAllButtons()
        
        # for button in all_buttons:
        #     button.clicked.connect()
        self.bt1.clicked.connect(self.Button1)
        self.bt2.clicked.connect(self.Button2)
        self.bt3.clicked.connect(self.Button3)
        self.bt4.clicked.connect(self.Button4)
        self.bt5.clicked.connect(self.Button5)
        self.bt6.clicked.connect(self.Button6)
        self.bt9.clicked.connect(self.ButtonReturn)
        
        self.center()
        self.show()
        
    def Button1(self):
        """
            Function:
                Make the System put the object in front of the mir base.
        """
        a = Empty()
        try:
            analyze_object_srv()
        except:
            pass

    def Button2(self):
        """
            Function:
                Make the System pick up the goal object using the UR5e
        """
        a = Empty()
        try:
            retrieve_object_srv()
        except:
            pass

    def Button3(self):
        """
            Function:
                Assemble the object using the UR16e.
        """
        assememble_object_srv()

    def Button4(self):
        """
            Function:
                Using the UR16e, disassemble the object.
        """
        disassemble_object_srv()
    
    def Button5(self):
        """
            Function:
                Send the MiR mobile base to the first goal.
        """
        goto_goal_srv(0)
        
    def Button6(self):
        """
            Function:
                Send the MiR mobile base to the second goal.
        """
        goto_start_srv(0)


    def ButtonReturn(self):
        """
            Function:
                Go to the main menu.
        """
        self.returnWindow()

    def Refresh(self):
        """
            Function:
                Refresh the window to update it based on the button pressed.
        """
        if self.count > 0:
            self.bt3.setText(str(self.count)+' seconds')
            self.count -= 1
        else:
            self.time.stop()
            self.bt3.setEnabled(True)
            self.bt3.setText('Button 3')
            self.count = 10
    def center(self):
        """
            Function:
                Center the main menu.
        """
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def createSingleButton(self,title,length,width,color):
        """
            Function:
                Function used to create a button of a color type.
            args:
                title - Title of the button
                length - The length of the button
                width - The width of the button
                color - The color of the button
        """
        button = QPushButton(title,self)
        button.resize(length, width)
        if color == "green":
            button.setStyleSheet("background-color : green")
        elif color == "red":
            button.setStyleSheet("background-color : red")
        elif color == "blue":
            button.setStyleSheet("background-color : blue")
        elif color == "orange":
            button.setStyleSheet("background-color : orange")

        return button
    def space_out(self,list,space,x,y):
        """
            Function:
                Create all the buttons on the current page of the GUI.
        """
        current_x = x
        for i in list:
            i.move(current_x,y)
            current_x += space
            
    def createAllButtons(self):
        """
            Function:
                Create all the buttons on the current page of the GUI.
        """
        length = 140
        width = 45
        self.bt1 = self.createSingleButton("Analyze",length,width,"green")
        self.bt2 = self.createSingleButton("Retrieve",length,width,"green")

        self.bt3 = self.createSingleButton("Assemble",140,width,"red")
        self.bt4 = self.createSingleButton("Disassemble",140,width,"red")
        self.bt5 = self.createSingleButton("Goto Goal",140,width,"orange")
        self.bt6 = self.createSingleButton("Goto Start",140,width,"orange")

        self.bt7 = self.createSingleButton("PlaceHolder ",140,width,"blue")
        self.bt8 = self.createSingleButton("PlaceHolder",140,width,"blue")
        self.bt9 = self.createSingleButton("Return To Main",140,width,None)

        self.bt10 = self.createSingleButton("5e Gripper Open",140,width,"green")
        self.bt11 = self.createSingleButton("5e Gripper Close",140,width,"green")
        self.bt12 = self.createSingleButton("16e Gripper Open",140,width,"green")
        self.bt13 = self.createSingleButton("16e Gripper Close",140,width,"green")
        row1 =  [self.bt1, self.bt2]
        row2 = [self.bt3, self.bt4, self.bt5, self.bt6]
        row3 = [ self.bt7, self.bt8, self.bt9]
        row4 = [ self.bt10, self.bt11, self.bt12,self.bt13]


        all_buttons = row1 + row2
        self.space_out(row1,space=150,x=200,y=50)
        #Second Row
        self.space_out(row2,space=150,x=50,y=100)

        #Third Row
        self.space_out(row3,space=150,x=50,y=150)
        self.space_out(row4,space=150,x=50,y=200)
        return all_buttons
    
    def returnWindow(self):                                             # <===
        """
            Function:
                Go to the home menu.
        """
        print('hi')
        self.w = Window()
        self.w.show()
        self.hide()
        
class Ui_MainWindow(QMainWindow):
    def __init__(self):
        """
        Create a window for pose or cartesian control
        """
        QMainWindow.__init__(self)
        self.setupUi(1)

    def setupUi(self, QMainWindow):
        """
            Function:
                Create the UI for the window.
        """
        self.resize(422, 355)
        self.centralwidget = QtWidgets.QWidget(self)

        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(160, 130, 93, 28))

        # For displaying confirmation message along with user's info.
        self.label = QtWidgets.QLabel(self.centralwidget)   
        self.label.setGeometry(QtCore.QRect(170, 0, 301, 111))

        # Keeping the text of label empty initially.      
        self.label.setText("")    

        self.setCentralWidget(self.centralwidget)
        self.retranslateUi(Window)
        QtCore.QMetaObject.connectSlotsByName(self)
    def returnWindow(self):                                             # <===
        """
            Function:
                Return to the main window
        """
        self.w = Window()
        self.w.show()
        self.hide()
    def retranslateUi(self, window):
        """
            Function:
                Refresh the window and add all the buttons.
        """
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Proceed"))
        self.pushButton.clicked.connect(self.takeinputs)

        self.pushConfirm = QPushButton("Confirm", self)
        self.pushConfirm.setText(_translate("MainWindow", "Send"))
        self.pushConfirm.clicked.connect(self.cartesian_Move)
        self.pushConfirm.move(250, 200)

        self.pushRetry = QPushButton("Input Again", self)
        self.pushRetry.setText(_translate("MainWindow", "Menu"))
        self.pushRetry.clicked.connect(self.returnWindow)
        self.pushRetry.move(50, 200)
        
    def takeinputs(self):
        """
            Function:
                Using values such as xyz, send the specified robot to the goal position.
        """
        name, done0 = QtWidgets.QInputDialog.getText(
            self, 'Input Dialog', 'Enter Robot Name:')
        pos_x, done1 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter X Position:')

        pos_y, done2 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Enter Y Position:') 

        pos_z, done3 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter Z Position:')

        orien_x, done4 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter X Orientation:')
        orien_y, done5 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter Y Orientation:')
        orien_z, done6 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter Z Orientation:')
        orien_w, done7 = QtWidgets.QInputDialog.getDouble(
            self, 'Input Dialog', 'Enter W Orientation:')

        if done1 and done2 and done3 and done7 :
            # Showing confirmation message along
            # with information provided by user.
            self.label.setText('Robot will go to the following '
                                '\nRobot Name: ' + str(name) +
                                '\nPosition X: ' + str(pos_x) + 
                                '\nPosition Y: ' + str(pos_y) + 
                                '\nPosition Z: ' + str(pos_z) + 
                                '\nOrientation X: ' + str(orien_x) + 
                                '\nOrientation Y: ' + str(orien_y) + 
                                '\nOrientation Z: ' + str(orien_z) + 
                                '\nOrientation W: ' + str(orien_w) )
            # Hide the pushbutton after inputs provided by the use.
            self.new_pose = [pos_x,pos_y,pos_z,orien_x,orien_y,orien_z,orien_w]
            self.name = name
            self.pushButton.hide()
    def cartesian_Move(self):
        """
            Function:
                Send the robot a pose goal and make the robot move there with a cartesian movement.
        """
        print(self.new_pose)
        a = list_to_pose(self.new_pose)
        send_pose = CPose
        send_pose.pose = a
        send_pose.name = self.name
        print(a)
        ur5e_step_srv(self.new_pose)
        print("Task Finished Successfully ")
        return Empty

    def get_EE_pos(self):
        """
            Function:
                Print the pose position of the robot.
        """
        ur_poses_srv
        return Empty


class Jsp_MainWindow(QMainWindow):
    """
    Create menu to control the robot using joint control.
    """
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(1)

    def setupUi(self, QMainWindow):
        """
            Function:
                Setup the UI for the window
        """
        self.resize(422, 255)
        self.centralwidget = QtWidgets.QWidget(self)

        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(160, 130, 93, 28))

        # For displaying confirmation message along with user's info.
        self.label = QtWidgets.QLabel(self.centralwidget)   
        self.label.setGeometry(QtCore.QRect(170, 40, 201, 111))

        # Keeping the text of label empty initially.      
        self.label.setText("")    

        self.setCentralWidget(self.centralwidget)
        self.retranslateUi(Window)
        QtCore.QMetaObject.connectSlotsByName(self)

    def retranslateUi(self, window):
        """
            Function:
                Refresh the window and add all the buttons.
        """
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Proceed"))
        self.pushButton.clicked.connect(self.takeinputs)

        self.pushConfirm = QPushButton("Confirm", self)
        self.pushConfirm.setText(_translate("MainWindow", "Send"))
        self.pushConfirm.clicked.connect(self.move_Joints)
        self.pushConfirm.move(250, 200)

        self.pushRetry = QPushButton("Input Again", self)
        self.pushRetry.setText(_translate("MainWindow", "Menu"))
        self.pushRetry.clicked.connect(self.returnWindow)
        self.pushRetry.move(50, 200)
        
    def takeinputs(self):
        """
            Function:
                Use the inputs in the dialog box to input desired values for each joint.
        """
        j1, done1 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 1')

        j2, done2 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 2')

        j3, done3 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 3')

        j4, done4 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 4')

        j5, done5 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 5')

        j6, done6 = QtWidgets.QInputDialog.getDouble(
        self, 'Input Dialog', 'Value for Joint 6')

        if done1 and done2 and done3 and done6 :
            # Showing confirmation message along
            # with information provided by user.
            self.label.setText('Robot will go to the following '
                                '\nJoint 1: ' + str(j1) + 
                                '\nJoint 2: ' + str(j2) + 
                                '\nJoint 3: ' + str(j3) + 
                                '\nJoint 4: ' + str(j4) + 
                                '\nJoint 5: ' + str(j5) + 
                                '\nJoint 6: ' + str(j6) ) 
            self.jointStates = [j1,j2,j3,j4,j5,j6]
        self.pushButton.hide()
    def move_Joints(self):
        """
            Function:
                Perform the joint movement.
        """
        print(self.jointStates)
        a = list_to_pose(self.jointStates)
        print(a)
        return self.jointStates 
# def joint_Control(joint_list):

    def returnWindow(self):                                             # <===
        """
            Function:
                Go to the main window.
        """
        self.w = Window()
        self.w.show()
        self.hide()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Window()
    rospy.init_node('GUIII11')
    
    ur5e_step_srv = rospy.ServiceProxy("marco/step", CPose)
    ur_poses_srv = rospy.ServiceProxy("marco/get_current_pose", Empty)

    analyze_object_srv = rospy.ServiceProxy("GUI/analyze_object", Empty)
    retrieve_object_srv = rospy.ServiceProxy("GUI/retrieve_object", Empty)
    disassemble_object_srv = rospy.ServiceProxy("GUI/disassemble_object", Empty)
    assememble_object_srv = rospy.ServiceProxy("GUI/assememble_object", Empty)
    ur5e_play_srv = rospy.ServiceProxy("/ur5e/ur_hardware_interface/dashboard/play", Trigger)
    ur16e_play_srv = rospy.ServiceProxy("/ur16e/ur_hardware_interface/dashboard/play", Trigger)

    ur5e_stop_srv = rospy.ServiceProxy("/ur5e/ur_hardware_interface/dashboard/stop", Trigger)
    ur16e_stop_srv = rospy.ServiceProxy("/ur16e/ur_hardware_interface/dashboard/stop", Trigger)

    ur5e_proRelease_srv = rospy.ServiceProxy("/ur5e/ur_hardware_interface/dashboard/unlock_protective_stop", Trigger)
    ur16e_proRelease_srv = rospy.ServiceProxy("/ur16e/ur_hardware_interface/dashboard/unlock_protective_stop", Trigger)

    ur5e_closePOP_srv = rospy.ServiceProxy("/ur5e/ur_hardware_interface/dashboard/close_safety_popup", Trigger)
    ur16e_closePOP_srv = rospy.ServiceProxy("/ur16e/ur_hardware_interface/dashboard/close_safety_popup", Trigger)

    goto_goal_srv = rospy.ServiceProxy("marco/moveto_goal", Empty)
    goto_start_srv = rospy.ServiceProxy("marco/moveto_start", Empty)

    gripper_5e_open_srv = rospy.ServiceProxy("GUI/5e_open", Empty)
    gripper_5e_close_srv = rospy.ServiceProxy("GUI/5e_close", Empty)
    gripper_16e_open_srv = rospy.ServiceProxy("GUI/16e_open", Empty)
    gripper_16e_close_srv = rospy.ServiceProxy("GUI/16e_close", Empty)

    sys.exit( app.exec_() )