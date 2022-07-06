#!/usr/bin/env python
import rospy    
import sys
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QGridLayout, QWidget, QMenu,QDesktopWidget
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QSize, QTimer

import os
import signal
import subprocess
import roslaunch
from LaunchRunClass import *


import time
import sys
path = sys.path[0]

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

import atexit

def exit_handler():
	kill_all()
atexit.register(exit_handler)

list_of_classes = []
def kill_all():
	print("Stoping all Processes...")
	for class_name in list_of_classes:
		class_name.stop()
	print("All Processes Stopped")


powerOnur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POn_UR5e")
list_of_classes.append(powerOnur5e)

def powerOn5e():
    print("Powering On UR5e")
    powerOnur5e.start()

powerOnur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POn_UR16e")
list_of_classes.append(powerOnur5e)

def powerOn16e():
    print("Powering On UR16e")
    powerOnur16e.start()
##
powerOffur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POff_UR5e")
list_of_classes.append(powerOffur5e)

def powerOff5e():
    print("Powering Off UR16e")
    powerOffur5e.start()

powerOffur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-POff_UR16e")
list_of_classes.append(powerOffur16e)

def powerOff16e():
    print("Powering Off UR16e")
    powerOffur16e.start()
##
BRur5e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-BR_UR5e")
list_of_classes.append(BRur5e)

def br5e():
    print("Powering Off UR16e")
    BRur5e.start()

BRur16e= CustomNodeRun("argonne_final_project", "hardwareSrcNode","-BR_UR16e")
list_of_classes.append(BRur16e)

def br16e():
    print("Powering Off UR16e")
    BRur16e.start()



class CreateGUi(QMainWindow):

    def __init__(self):
        QMainWindow.__init__(self)

        self.setMinimumSize(QSize(700, 350))    
        self.setWindowTitle("VR Baxter Demo") 
        
        all_buttons = self.createAllButtons()
        
        # for button in all_buttons:
        #     button.clicked.connect()
        self.bt1.clicked.connect(self.Button1)
        self.bt2.clicked.connect(self.Button2)
        self.bt3.clicked.connect(self.Button3)
        self.bt4.clicked.connect(self.Button4)
        self.bt5.clicked.connect(self.Button5)
        self.bt6.clicked.connect(self.Button6)

        self.center()

        self.show()
        

    def Button1(self):
        powerOn5e()
        
    def Button2(self):
        powerOn16e()

    def Button3(self):
        powerOff5e()
        
    def Button4(self):
        powerOff16e()
    
    def Button5(self):
        br5e()
        
    def Button6(self):
        br16e()

    def Refresh(self):
        if self.count > 0:
            self.bt3.setText(str(self.count)+' seconds')
            self.count -= 1
        else:
            self.time.stop()
            self.bt3.setEnabled(True)
            self.bt3.setText('Button 3')
            self.count = 10
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def createSingleButton(self,title,length,width,color):
        button = QPushButton(title,self)
        button.resize(length, width)
        if color == "green":
            button.setStyleSheet("background-color : green")
        elif color == "red":
            button.setStyleSheet("background-color : red")

        return button
    def space_out(self,list,space,x,y):
        current_x = x
        for i in list:
            i.move(current_x,y)
            current_x += space
            
    def createAllButtons(self):
        length = 140
        width = 45
        self.bt1 = self.createSingleButton("PowerOn UR5e",length,width,"green")
        self.bt2 = self.createSingleButton("PowerOn UR16e",length,width,"green")

        self.bt3 = self.createSingleButton("PowerOff UR5e",140,width,"red")
        self.bt4 = self.createSingleButton("PowerOff UR16e",140,width,"red")
        self.bt5 = self.createSingleButton("BrakeRel UR5e",140,width,None)
        self.bt6 = self.createSingleButton("BrakeRel UR16e",140,width,None)

        self.bt7 = self.createSingleButton("Load Script UR5e ",140,width,None)
        self.bt8 = self.createSingleButton("Load Script UR16e",140,width,None)
        self.bt9 = self.createSingleButton("Start R-Controller",140,width,None)

        row1 =  [self.bt1, self.bt2]
        row2 = [self.bt3, self.bt4, self.bt5, self.bt6]
        row3 = [ self.bt7, self.bt8, self.bt9]

        all_buttons = row1 + row2
        # row4 = [ self.bt10, self.bt11]

        # self.bt1.move(50,50)
        # self.bt2.move(150,50)
        self.space_out(row1,space=150,x=200,y=50)
        #Second Row
        # self.bt3.move(50,100)
        # self.bt4.move(210,100)
        # self.bt5.move(360,100)
        # self.bt6.move(510,100)
        self.space_out(row2,space=150,x=50,y=100)

        #Third Row
        # self.bt7.move(50,150)
        # self.bt8.move(250,150)
        # self.bt9.move(450,150)
        self.space_out(row3,space=150,x=50,y=150)
        #Last Row
        return all_buttons

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = CreateGUi()
    mainWin.show()
    sys.exit( app.exec_() )