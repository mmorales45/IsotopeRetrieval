#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <string.h>
#include <cstring>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <std_srvs/Empty.h>
#include <argonne_final_project/GripperName.h>


class MotionPlanning
{
    public:
        MotionPlanning(std::string ur5e_plan,std::string ur16e_plan)
        {
            gripper_state_srv = nh.advertiseService("/MotionPlanning/gripper_state", &MotionPlanning::gripper_state, this);
            secs =ros::Time::now().toSec();
        }
        bool gripper_state(argonne_final_project::GripperName::Request& req, argonne_final_project::GripperName::Response& res)
        {   
            std::string gripper_name = req.name; std::string state = req.state;
            std::string ur5e = "ur5e"; std::string ur16e = "ur16e";
            std::string close = "close"; std::string open = "open";
            if (strcasecmp(close.c_str(), state.c_str()) == 0){
                ROS_INFO_STREAM("Opening Gripper on " << gripper_name);
            }
            else if (strcasecmp(open.c_str(), state.c_str()) == 0){
                ROS_INFO_STREAM("Closing Gripper on " << gripper_name);
            }
            else{
                ROS_INFO_STREAM("Please Input either a close or open command!");
            }
            double b = ros::Time::now().toSec();
            std::cout << "Current time" << b << std::endl;
            std::cout << b - secs << std::endl;
            res.success = true;
            return true;
        }


    private:
    ros::NodeHandle nh;
    // moveit::planning_interface::MoveGroupInterface move_group_ur5e;
    // moveit::planning_interface::MoveGroupInterface move_group_ur16e;
    ros::ServiceServer gripper_state_srv;

    double secs;

};


int main(int argc, char * argv[])
{
    //Create node.
    ros::init(argc, argv, "target_handle");
    //Define the names of the planning groups.
    std::string UR5e_GROUP = "ur5e";
    std::string UR16e_GROUP = "ur16e";
    
    MotionPlanning node = MotionPlanning(UR5e_GROUP,UR16e_GROUP);
    //Make sure the node does not end.
    ros::spin();
    return 0;
}