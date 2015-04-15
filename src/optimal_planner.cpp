//ROS
#include <ros/ros.h>

//Server
#include "optimal_planner/PathPlan.h"

//OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//OMPL Planners
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

//Robot Model
#include "motoman_moveit.hpp"

//Trajectory
#include <moveit/robot_trajectory/robot_trajectory.h>

//MOVEIT
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

//Output
#include <iostream>

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

enum planner_type
{
    RRT_STAR,
    RRT_Connect
};

#define Workspace_X_Limits 1.5 //meters
#define Workspace_Y_Limits 2.0 //meters
#define Workspace_Z_Limits 2.0 //meters

namespace ob = ompl::base ;
namespace og = ompl::geometric ;

/*
 * Dummy deallocation functions when creating smart pointers from pointers to RAII'd objects rather than new'd objects
 */
void dealocate_StateValidityChecker_fn(ompl::base::StateValidityChecker* p)
{
    std::cout << ">>>>>>>>>>>>>>>Deallocate>>>>>>>>>>>>" << std::endl ;
    UNUSED(p);
}

void dealocate_MotionValidator_fn(ompl::base::MotionValidator* p)
{
    UNUSED(p);
}

void dealocate_OptimiztionObjective_fn(ompl::base::OptimizationObjective* p)
{
    UNUSED(p);
}


class ValidityChecker : public ob::StateValidityChecker
{
protected:

    std::unique_ptr<planning_scene::PlanningScene> planning_scene_ptr_;

public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si)
    {
        robot_model_loader::RobotModelLoader model_loader("robot_description");
        planning_scene_ptr_.reset();
        planning_scene_ptr_ = std::unique_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(model_loader.getModel()));
        specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::NONE;
        specs_.hasValidDirectionComputation = false;
    }

    //Function to check if state is valid
    virtual bool isValid(const ob::State *state) const
    {
        std::cout << ">>>>>>>>>>>>>>>Validity Check>>>>>>>>>>>>" << std::endl ;
        std::vector<double> JointValues(7) ;
        const ob::RealVectorStateSpace::StateType *sample_state = state->as<ob::RealVectorStateSpace::StateType>() ;
        JointValues[0] = sample_state->values[0] ;
        JointValues[1] = sample_state->values[1] ;
        JointValues[2] = sample_state->values[2] ;
        JointValues[3] = sample_state->values[3] ;
        JointValues[4] = sample_state->values[4] ;
        JointValues[5] = sample_state->values[5] ;
        JointValues[6] = sample_state->values[6] ;

        robot_state::RobotState& current_state = planning_scene_ptr_->getCurrentStateNonConst();
        const robot_model::JointModelGroup* model_group = current_state.getJointModelGroup("arm_left");
        current_state.setVariablePositions(JointValues);
        if(planning_scene_ptr_->isStateValid(current_state, "left_arm") && current_state.satisfiesBounds(model_group))
        {
            std::cout << ">>>>>>>>>>>>>>>Valid State>>>>>>>>>>>>" << std::endl ;
            return true  ;
        }
        else
        {
            std::cout << ">>>>>>>>>>>>>>>Invalid State>>>>>>>>>>>>" << std::endl ;
            return false  ;
        }
    }

};

//Plan Class
namespace optimal_planner
{
class motoman_planner
{
public:
    motoman_planner()
    {
        // Give default planning time
        planning_time = 10;
        goal_tolerance = 0.01;
        cost_bias = 0.5;
        planner_choice = RRT_STAR;
    }
    bool start_planning()
    {
        //std::vector<double> start_Config;
        //std::vector<double> goal_Config;

        ob::PathPtr path;

        //Construct the robot state space
        ob::StateSpacePtr r_space(new ob::RealVectorStateSpace(motoman_arm_DOF)) ;
        ob::RealVectorBounds Joint_bounds(motoman_arm_DOF) ;

        //Joint bounds
        setStateSpaceLimits(Joint_bounds) ;
        r_space->as<ob::RealVectorStateSpace>()->setBounds(Joint_bounds);

        //Setup sample space
        ob::SpaceInformationPtr sample_si(new ob::SpaceInformation(r_space)) ;

        //Setup Validity Checker
        ValidityChecker checker(sample_si) ;
        ompl::base::StateValidityCheckerPtr validity_checker(&checker, dealocate_StateValidityChecker_fn);
        sample_si->setStateValidityChecker(validity_checker);
        sample_si->setStateValidityCheckingResolution(0.03); // 3%
        sample_si->setup();

        //Set start and end state
        ob::ScopedState<> start_state = SetStateConfig(start_Config,sample_si) ;
        start_state.print(std::cout) ;
        ob::ScopedState<> goal_state= SetStateConfig(goal_Config,sample_si) ;
        goal_state.print(std::cout) ;

        //Create Problem Definition
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(sample_si)) ;
        pdef->setStartAndGoalStates(start_state,goal_state) ;

        //Make the planner
        ob::PlannerPtr planner(new og::RRTstar(sample_si)) ;

        planner->setProblemDefinition(pdef) ;
        planner->setup() ;

        //Planning
        ob::PlannerStatus solved = planner->solve(planning_time) ;
        if(solved)
        {
            //Generate the path
            path = pdef->getSolutionPath() ;
            std::cout<< "Path Found" << std::endl ;
            path->print(std::cout) ;
            return true ;
        }
        else
        {
            std::cout << "Can't find solution" << std::endl ;
            return false ;
        }
    }

    bool setStateSpaceLimits(ob::RealVectorBounds& Joint_bounds)
    {
        // JointSpace Limits
        Joint_bounds.setLow(0,-Joint_1_s_Limits);
        Joint_bounds.setLow(1,-Joint_2_l_Limits);
        Joint_bounds.setLow(2,-Joint_3_e_Limits);
        Joint_bounds.setLow(3,-Joint_4_u_Limits);
        Joint_bounds.setLow(4,-Joint_5_r_Limits);
        Joint_bounds.setLow(5,-Joint_6_b_Limits);
        Joint_bounds.setLow(6,-Joint_7_t_Limits);

        Joint_bounds.setHigh(0,Joint_1_s_Limits);
        Joint_bounds.setHigh(1,Joint_2_l_Limits);
        Joint_bounds.setHigh(2,Joint_3_e_Limits);
        Joint_bounds.setHigh(3,Joint_4_u_Limits);
        Joint_bounds.setHigh(4,Joint_5_r_Limits);
        Joint_bounds.setHigh(5,Joint_6_b_Limits);
        Joint_bounds.setHigh(6,Joint_7_t_Limits);

        return true;
    }

    ob::ScopedState<> SetStateConfig(std::vector<double>& state_config, ob::SpaceInformationPtr smp_space)
    {
        ob::ScopedState<> rt_state(smp_space);
        if(state_config.size())
        {
            std::vector<double>::iterator iter = state_config.begin();
            for(int i = 0;iter!= state_config.end();iter++,i++)
            {
                rt_state->as<ob::RealVectorStateSpace::StateType>()->values[i] = *iter;
            }
        }

        return rt_state;
    }

public:
    float planning_time;
    double goal_tolerance;
    planner_type planner_choice;
    float cost_bias;

    std::vector<double> start_Config;
    std::vector<double> goal_Config;

    ob::PathPtr path;
} ;
}

//Instantiate the planner
optimal_planner::motoman_planner* m_planner ;
motoman_move_group* m_robot_model ;

//Planning call
bool plan(optimal_planner::PathPlan::Request &req, optimal_planner::PathPlan::Response &res)
{
    std::vector<double> goal_test(7,0.1) ;

    //Using current config as start
    m_planner->start_Config.clear() ;
    m_planner->start_Config = m_robot_model->GetGroupConfig(req.group_name) ;

    //Assign Goal Config
    m_planner->goal_Config.clear() ;
    if(req.target_config.empty())
    {
        std::cout<< "No Target Assigned. Exit!" <<std::endl ;
        return false ;
    }

    std::vector<double> temp_target_config ;
    for(int i=0 ; i<req.target_config.size() ; i++)
    {
        double JointValue = req.target_config[i] ;
        temp_target_config.push_back(JointValue) ;
    }

    m_planner->goal_Config = temp_target_config ;
    std::vector<double>::iterator goal_iter = m_planner->goal_Config.begin() ;
    std::cout << ">>>>>>>>>>>>>>>Goal Config>>>>>>>>>>>>" << std::endl ;
    for(int i=1 ; goal_iter!=m_planner->goal_Config.end() ; goal_iter++,i++)
    {
        std::cout << "Config" << i <<": "<< *goal_iter << std::endl ;
    }

    //Planning time
    m_planner->planning_time = 5 ;

    //Start Planner
    m_planner->start_planning() ;

    return true ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Optimal_plan_server") ;
    ros::NodeHandle n ;
    ros::ServiceServer server = n.advertiseService("Optimal_plan_server",plan) ;
    m_robot_model = new motoman_move_group ;

    ROS_INFO("Robot Model Loaded!") ;
    m_planner = new optimal_planner::motoman_planner ;
    ROS_INFO("Optimal planner created!") ;

    ROS_INFO(">>>>>>>>>>>>>>> Plan server created! >>>>>>>>>>>>>>>") ;

    ros::spin() ;
    return 0 ;
}
