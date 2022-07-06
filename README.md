# Argonne Dual Arm Mobile Base Retrieval System
By Marco Morales

<!-- ![Robot_first](pictures/ArgonnePic1.png)

![Robot_second](pictures/ArgonnePic2.png) -->

## Dependencies

Robotiq For Noetic 
```
git clone -b noetic-devel git@github.com:jr-robotics/robotiq.git
```



Instructions for the Mobile Base to Move

1. Turn on the robot and ensure the knob position is set to Automatic, manual should be used for pushing the robot to a desired position.

2. Connect to the MiR website.
    - For Marco, the username is `Marco` and the password is `test`.
    - If you plan on being a long term user, create a new user account through an existing account.

3. The following substeps are useful for preventing errors and headaches down the road.
    - Ensure the robot's time and the computers times are in sync. This can be done by going into System > Settings > Data & Time and then clicking on laod from device followed by `Save Changes`. Wait for the system to sync up. 

4. On you computer, load the following launchfile.
    ```
    roslaunch mir_driver mir.launch
    ``` 
    - This will establish a connect between the MiR's and your computer. This may take a few seconds so please be patient. 

5. Now run the hector mapping launchfile to map the area around the robot.
    ```
    roslaunch mir_navigation hector_mapping.launch
    ```

6. Now run the move_base launchfile which will enable the mobile base to be controlled thourgh move base goals.
    ```
    roslaunch mir_navigation move_base.xml with_virtual_walls:=false
    ```
7. Finally, run the RVIZ file to see visual representaion of the operations.
    ```
    rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
    ```
8. Now the map should be visible to you, if not try zooming out to see if the robot and map were offset and then move the camera to that position. 
    - You can set waypoints/goals by clicking on `Set 2D nav goal` and then clicking on a location and the  releasing at the disired orientation. 

Accessing cameras of MiR base

1. export ROS_MASTER_URI=http://192.168.12.20:11311

2. rqt_image_view


## For Single Arm Control

### UR5e
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.12.248 s
rosservice call /ur_hardware_interface/dashboard/brake_release
rosservice call /ur_hardware_interface/dashboard/play
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
rosrun argonne_final_project test_node
rosservice call /step_left
```

## Controlling Both Arms from One Computer

```
roslaunch argonne_final_project launch_all.launch
```

## Two routes Pendant or terminal

For both parts, it is required to make sure the loaded script has the external control module included and this requires the IP of the machine it is connected to in order to function.
### Pendant 

On each robot, start the robots from off to running. Bottom right corner -> start 

Then press play on the run tab of the program. This loads the last script that was loaded for each arm. If the the script is void or not externalControl, find it and run it.

### All from Terminal

For each arm, start the robot by running the services below but make sure that before running the second file, that the first one reaches RUNNING mode.
```
rosservice call /UR5e/ur_hardware_interface/dashboard/brake_release
rosservice call /UR16e/ur_hardware_interface/dashboard/brake_release
```

Then press play to run the last script for each arm.

```
rosservice call /UR5e/ur_hardware_interface/dashboard/play
rosservice call /UR16e/ur_hardware_interface/dashboard/play
```

To pause the arms, run the services below

```
rosservice call /UR5e/ur_hardware_interface/dashboard/pause
rosservice call /UR16e/ur_hardware_interface/dashboard/pause
```

To power off the robot, run the services below

```
rosservice call /UR5e/ur_hardware_interface/dashboard/power_off
rosservice call /UR16e/ur_hardware_interface/dashboard/power_off
```
There are two ways to control the gripper, using a python or C++ library.

