#include <ros/ros.h>
#include "optimal_planner/PathPlan.h"

// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Move Group
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

using namespace move_group_interface;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimal_Planning_Client");

    ros::NodeHandle nh;
    ros::ServiceClient Client = nh.serviceClient<optimal_planner::PathPlan>("Optimal_plan_server");
    optimal_planner::PathPlan srv;

    srv.request.group_name = "arm_left";


    // -0.3772088970063918, -1.3143374665077188, 1.3055896989571494, -1.2431863366129394, -2.9914519460193256, 1.3527307468837322, 1.6504463451922884
    float Jnt_Val_s = -0.3772088970063918;
    float Jnt_Val_l = -1.3143374665077188;
    float Jnt_Val_e =  1.3055896989571494;
    float Jnt_Val_u = -1.2431863366129394;
    float Jnt_Val_r = -2.9914519460193256;
    float Jnt_Val_b =  1.3527307468837322;
    float Jnt_Val_t =  1.6504463451922884;

    srv.request.target_config.push_back(Jnt_Val_s);
    srv.request.target_config.push_back(Jnt_Val_l);
    srv.request.target_config.push_back(Jnt_Val_e);
    srv.request.target_config.push_back(Jnt_Val_u);
    srv.request.target_config.push_back(Jnt_Val_r);
    srv.request.target_config.push_back(Jnt_Val_b);
    srv.request.target_config.push_back(Jnt_Val_t);

    //Eigen::Affine3d start_pose = kinematic_state->getGlobalLinkTransform(leftArm->getEndEffectorName());
    //move_group_interface::MoveGroup right_arm_group("arm_right");
    //move_group_interface::MoveGroup left_arm_group("arm_left");

    //geometry_msgs::PoseStamped left_ee_pose = left_arm_group.getCurrentPose(left_arm_group.getEndEffector());

    //srv.request.start.position.x = left_ee_pose.pose.position.x;
    //srv.request.start.position.y = left_ee_pose.pose.position.y;
    //srv.request.start.position.z = left_ee_pose.pose.position.z;
    //srv.request.start.orientation.x  =left_ee_pose.pose.orientation.x;
    //srv.request.start.orientation.y  =left_ee_pose.pose.orientation.y;
    //srv.request.start.orientation.z  =left_ee_pose.pose.orientation.z;
    //srv.request.start.orientation.w  =left_ee_pose.pose.orientation.w;

    //ROS_INFO("Start Postion: x = %f, y = %f, z = %f",srv.request.start.position.x, srv.request.start.position.y, srv.request.start.position.z);

    /*srv.request.target.x = 0.47674;
    srv.request.target.y = 0.00001;
    srv.request.target.z = 0.94136;

    srv.request.target.x = 0.2;
    srv.request.target.y = 0.2;
    srv.request.target.z = 0.2;

    srv.request.time_limit = 10;

    srv.request.p = 0.5;
    srv.request.planner_type = deformable_ompl::PlanPath::Request::RRTSTAR;
    srv.request.target_tolerance = 0.01;*/

    if(Client.call(srv))
    {
        ROS_INFO("Found! Test Complete!");
        ROS_INFO("Total Length is: %f", srv.response.total_length);
        ROS_INFO("Total Cost is: %f", srv.response.total_cost);
    }
    else
    {
        ROS_ERROR("Failed to call service!");
        return 1;
    }

    return 0;
}

