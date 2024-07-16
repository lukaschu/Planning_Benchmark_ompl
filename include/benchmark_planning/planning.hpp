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

#include <moveit_msgs/msg/robot_trajectory.h>
#include <moveit_msgs/msg/robot_state.h>

using moveit::planning_interface::MoveGroupInterface;

namespace ob = ompl::base;
namespace oc = ompl::control;

class Planning : public rclcpp::Node
{
public:
    Planning(rclcpp::Node *parent_node);
    static void dynamics(const ob::State *start, const oc::Control *control, const double duration, ob::State *end); // Defining the state transition
    void call_scenario_loader(std::string scenario); // is called to establish the whole scenario and environemnt
    void set_initial(ob::ScopedState<> *initial,ob::ScopedState<> *final, std::vector<double> initial_state, std::vector<double> final_state); // conditions are set for planner
    void solve(std::vector<double> initial_state, std::vector<double> true_start_state_pos, std::vector<double> final_state, ob::PathPtr &path); // main solve loop
    void solve_with_moveit(); // if desired, one can also use the moveit planner to estbalish a plan
    MoveGroupInterface::Plan recover_moveit_path(ob::PathPtr &path, std::vector<double> true_start_state_pos, double duration, ob::State *start); // ompl plan is rewritten into a moveit readable executable plan
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr debug_publisher_; // Publisher to debug

private:
    std::shared_ptr<ob::CompoundStateSpace> space_; // defines config. space (q,q_dot)
    std::shared_ptr<oc::RealVectorControlSpace> ctrl_space_; // defines input/ctrl space (acccel. = q_dot_dot)
    std::shared_ptr<oc::SpaceInformation> si_; // space information
    MoveGroupInterface move_group_interface_; // Interface needed for trajectory execution
    std::shared_ptr<Collision_Checker> collision_checker_; // contains class which rules the collision check
    const double pi_ = 3.14159;
    rclcpp::Node* main_node_;
};

#endif