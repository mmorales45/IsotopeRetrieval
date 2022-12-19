#!/usr/bin/env python
##Needs python3 to work
#python3 needs tkinter, python 2.7 needs Tkinter

"""
    MiR node to provide services to the manipulation node to send the mobile base to goals.
    SERVICES:
        marco/moveto_goalA (std_srvs/Empty) - Send robot to Goal A
        marco/moveto_goalB (std_srvs/Empty) - Send robot to Goal B
"""
import sys

import rospy
from std_srvs.srv import Empty
import requests, json
from Tkinter import *

ip = '192.168.1.2' # Ip if not connected to robot itself
# host = 'http://mir.com/api/v2.0.0/'   #If connected to robot
host = 'http://' + ip + '/api/v2.0.0/'   # if not connected to robot

class mobile_base:
    """Create services that interact with the MiR 250"""
    def __init__(self): 
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic OmUzYjBjNDQyOThmYzFjMTQ5YWZiZjRjODk5NmZiOTI0MjdhZTQxZTQ2NDliOTM0Y2E0OTU5OTFiNzg1MmI4NTU='
        self.mission_service_end = rospy.Service("marco/moveto_goalA", Empty,self.Mission_Start)
        self.mission_service_start = rospy.Service("marco/moveto_goalB", Empty,self.Mission_End)
        get_missions = requests.get(host + 'missions',headers = self.headers)
        print(get_missions)
        

    def Mission_Start(self,req):
        """
            Function:
                Send the mission id corresponding to Goal A
            Args:
                req - Empty
        """
        start = {"mission_id": "aae45adf-d82a-11ec-84ed-00012978eb25"}
        post_mission = requests.post(host + 'mission_queue', json = start, headers = self.headers)
    
    def Mission_End(self,req):
        """
            Function:
                Send the mission id corresponding to Goal B
            Args:
                req - Empty
        """
        end = {"mission_id": "bsf45oem-d62g-66gh-93ed-00045873rt76"}
        post_mission = requests.post(host + 'mission_queue', json = end, headers = self.headers)
    
    


def main(): #The main function.
    rospy.init_node("mir_NAVIGATOR")
    try:
        mission_move = mobile_base()
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
Footer