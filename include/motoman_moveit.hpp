// ROS
#include <ros/ros.h>
// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
// Move Group
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

// Manipulator DOF
#define motoman_arm_DOF 7

// motoman_joint_limits
#define Joint_1_s_Limits 3.13
#define Joint_2_l_Limits 1.90
#define Joint_3_e_Limits 2.95
#define Joint_4_u_Limits 2.36
#define Joint_5_r_Limits 3.13
#define Joint_6_b_Limits 1.90
#define Joint_7_t_Limits 3.13

class motoman_move_group
{
public:
    motoman_move_group()
    {
        right_arm_group = new move_group_interface::MoveGroup("arm_right");
        left_arm_group = new move_group_interface::MoveGroup("arm_left");
    }



    std::vector<double> GetGroupConfig(std::string& groupName)
    {
        std::vector<double> JointValues;
        if (groupName == right_arm_group->getName())
        {
            JointValues = right_arm_group->getCurrentJointValues();
        }
        else if(groupName == left_arm_group->getName())
        {
            JointValues = left_arm_group->getCurrentJointValues();
        }
        else
        {
            ROS_INFO("Invalid GroupName");
        }

        return JointValues;
    }

public:
    move_group_interface::MoveGroup* right_arm_group;
    move_group_interface::MoveGroup* left_arm_group;

};
