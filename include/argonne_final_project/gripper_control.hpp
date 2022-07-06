#ifndef GRIPPER_CONTROL_INCLUDE_GUARD_HPP
#define GRIPPER_CONTROL_INCLUDE_GUARD_HPP

/// \file 
/// \brief Library for controlling a Robotiq Gripper


#include <iosfwd>
#include <iostream>
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
namespace argonne_final_project
{

    class gripperControl 
    {
        public:
            /// \brief Create an object of the class.
            gripperControl();

            /// \brief Reset the gripper's values to 0.
            void reset();
            
            /// \brief Activate the gripper and initialize the parameters.
            void activate();

            /// \brief Open the gripper completely.
            void open();

            /// \brief Close the gripper completely.
            void close();

            /// \brief Increase the speed at which the gripper moves.
            /// \param increment- The increase in speed. 
            void increase_speed(int increment);

            /// \brief Decrease the speed at which the gripper moves.
            /// \param increment- The decrease in speed. 
            void decrease_speed(int increment);
            
            /// \brief Increase the force at which the gripper moves.
            /// \param increment- The increase in force. 
            void increase_force(int increment);

            /// \brief Decrease the force at which the gripper moves.
            /// \param increment- The decrease in force. 
            void decrease_force(int increment);

        private:
            robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;
            
    };
}

#endif 