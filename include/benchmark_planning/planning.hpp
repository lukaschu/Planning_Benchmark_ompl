#ifndef PLANNING
#define PLANNING

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <iostream>

#include "benchmark_planning/collision_checker.hpp"


#include "rclcpp/rclcpp.hpp"
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Path.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using moveit::planning_interface::MoveGroupInterface;

#include <moveit_msgs/msg/robot_trajectory.h>
#include <moveit_msgs/msg/robot_state.h>

//#include <ompl/base/RealVectorBounds.h>
//#include <ompl/base/CompoundStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class Planning : public rclcpp::Node
{
public:
    Planning();
    static void dynamics(const ob::State *start, const oc::Control *control, const double duration, ob::State *end);
    void call_scenario_loader();
    void set_initial(ob::ScopedState<> *initial,ob::ScopedState<> *final, std::vector<double> initial_state, std::vector<double> final_state);
    void solve(std::vector<double> initial_state, std::vector<double> final_state, ob::PathPtr &path);
    void solve_with_moveit();
    MoveGroupInterface::Plan recover_moveit_path(ob::PathPtr &path, double duration, ob::State *start);
    //rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_; // publisher for collision scene

    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr debug_publisher_; // Publisher

private:
    std::shared_ptr<ob::CompoundStateSpace> space_; // defines config. space (q,q_dot)
    std::shared_ptr<oc::RealVectorControlSpace> ctrl_space_; // defines input/ctrl space (acccel. = q_dot_dot)
    std::shared_ptr<oc::SpaceInformation> si_;
    std::string solver_; // Defines the sampling algo. that is used (RRT*, Kpiece ....)
    MoveGroupInterface move_group_interface_;
    const double pi_ = 3.14159;
    std::shared_ptr<Collision_Checker> collision_checker_;
};

#endif