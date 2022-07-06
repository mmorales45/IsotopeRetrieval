#include <argonne_final_project/gripper_control.hpp>
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"

/*
Note the follow argument descriptions are for the Output message of the gripper.
rACT: Activate the Gripper
rGTO: Go to Position Status
rATR: Automatic Release
rPR: Target Position for Gripper
rSP: Target Speed for Gripper
rFR: Target Force for Gripper
*/

namespace argonne_final_project
{
    gripperControl::gripperControl()
    {
        reset();
    }

    void gripperControl::reset()
    {
        command.rACT = 0;
        command.rGTO = 0;
        command.rATR = 0;
        command.rPR = 0;
        command.rSP = 0;
        command.rFR = 0;   
    }
    void gripperControl::activate()
    {
        command.rACT = 1;
        command.rGTO = 100;
        command.rSP = 255;
        command.rFR =150;
    }

    void gripperControl::open()
    {
        command.rPR = 0;
    }

    void gripperControl::close()
    {
        command.rPR = 255;
    }

    void gripperControl::increase_speed(int increment)
    {
        command.rSP += increment;
        if (command.rSP > 255)
        {
            command.rSP = 255;
        }
    }

    void gripperControl::decrease_speed(int increment)
    {
        command.rSP -= increment;
        if (command.rSP < 0)
        {
            command.rSP = 0;
        }
    }

    void gripperControl::increase_force(int increment)
    {
        command.rFR += increment;
        if (command.rFR > 256)
        {
            command.rFR = 255;
        }
    }

    void gripperControl::decrease_force(int increment)
    {
        command.rFR -= increment;
        if (command.rFR < 0)
        {
            command.rFR = 0;
        }
    }
}