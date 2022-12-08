# Argonne Dual Arm Mobile Base Retrieval System
By Marco Morales
<!-- Need to resize the pictures below -->
<!-- ![Robot_first](pictures/ArgonnePic1.png)

![Robot_second](pictures/ArgonnePic2.png) -->

## Prerequisites

To download the packages, please install vcstool using the following link

[vcstool github link](https://github.com/dirk-thomas/vcstool)

Once completed, import the .repos file using the following command line in the src folder of your workspace.

```
vcs import < DualArmMIR.repos
```

Next install any dependencies the packages have using the commands below.

```
sudo apt-get install ros-$ROS_DISTRO-moveit-visual-tools
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
# For Single Arm Control

### UR5e Example

Regardless of which arm you want to control, run the first command below but with the correct UR type and IP address, ex. ur16e_bringup.launch. The services to power on, play the external control script are the same. Make sure to also run the correct moveit_planning_exection.launch file by appending the correct arm, ex. ur16e_.

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.12.248
rosservice call /ur_hardware_interface/dashboard/brake_release
rosservice call /ur_hardware_interface/dashboard/play
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
rosrun argonne_final_project test_node
rosservice call /step_left
```

## Controlling Both Arms from One Computer

The launch_all launch file spawns drivers for both arms in a single executable. Make sure to use the correct ip addresses for each arm. 

```
roslaunch argonne_final_project launch_all.launch UR16e_ip:=192.168.12.248 UR5e_ip:=192.168.12.242
```

The following launch file will run the MoveIt launchfile to control both arms through MoveIt API or through RVIZ.

```
roslaunch dual_arm_moveit_config move_group.launch
```

## Launching the ExternalControl Script

A very important note of the Universal Robots systems is that in order to control the arms through ROS, the ExternalControl script has to play. There are two ways to launch the script, either through the teach pendant or through the command line.

For both parts, make sure the loaded script has the external control module included and this requires the IP of the machine it is connected to in order to function.

### Pendant 

Power on the robot. Once everything is done booting up, move to the bottom right corner and click on `Start`. Now on the new screen, press `Turn On` and then `Release`. Now press `Exit` and then press play on the run tab of the program. 

### Command Line

For each arm, power on the robot and make sure to run the launch file that runs the driver for the respective arm. Once the drivers launch successfully, start the robot by running the service below. This may take a second or two since it is powering on the robot and releasing the brakes as well.
```
rosservice call /ur_hardware_interface/dashboard/brake_release
```

Then press play to run the last loaded script for each arm, which in this case should be the ExternalControl script.

```
rosservice call /ur_hardware_interface/dashboard/play
```

# Running the Demo

1. Power on the UR16e, UR5e, and MiR 250 mobile base. 

2. Run the drivers for both robotic arms.
    ```
    roslaunch argonne_final_project dual_arm_bringup.launch
    ```
    
3. Play the ExternalControl scripts for both arms. Refer to above to play the script on each arm through the teach pendant or command line. 

4. Run the main launch file that will run the nodes for the AprilTags, Intel RealSense camera, and main manipulation node.
    ```
    roslaunch argonne_final_project mock_demo.launch
    ```
    
5. The arms should now tuck in to prevent collision when moving around on the base. The robotic system should now move to the first goal point, uncouple and disassemble, move to the second goal point and then assemble.


## Instructions for the Mobile Base ROS Drivers

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
```
export ROS_MASTER_URI=http://192.168.12.20:11311
rqt_image_view
```

# TroubleShooting/Common Issues

## ROS Drivers Run But No Control

For this issue, one of the known errors if you programmed the robot to run without a teach pendant but for some reason, one is installed. Make sure to go into settings, enter the password (most likely it will be "easybot"), and then change the settings to be correct for the current setup.

## ROS Drivers Fail

There are a number of reasons this could occur. Some of the common issues are the IP addresses for the arms do not align with those chosen. Try disabling any wifi connection on the laptop and instead make sure the only active connection is the ethernet connection to both robotic arms. 
