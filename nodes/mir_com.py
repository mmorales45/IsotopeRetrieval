from urllib import request
import requests, json
from tkinter import *

#Request
ip = '192.168.12.1' # Ip if not connected to robot itself
host = 'http://mir.com/api/v2.0.0/'   #If connected to robot
# host = 'http://' + ip + '/api/v2.0.0/'   # if not connected to robot

#headers
headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = 'Basic bWFyY286OWY4NmQwODE4ODRjN2Q2NTlhMmZlYWEwYzU1YWQwMTVhM2JmNGYxYjJiMGI4MjJjZDE1ZDZjMTViMGYwMGEwOA=='

print(headers)

get_missions = requests.get(host + 'missions',headers = headers)
print(get_missions.text)

mission_id = {"mission_id": "aae45adf-d82a-11ec-84ed-00012978eb25"}
post_mission = requests.post(host + 'mission_queue', json = mission_id, headers = headers)