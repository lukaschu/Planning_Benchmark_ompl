#ifndef SIMULATION
#define SIMULATION

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "benchmark_planning/planning.hpp"
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class Simulation : public rclcpp::Node
{   
public:
    Simulation();
    void run_simulation();
    void get_current_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
private:
    Planning planner;
    ob::PathPtr path; // will contain the relevant path
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_subscription_; // Subscriber
    std::vector<double> initial_state_;
    bool state_received_ = false;
    const double pi_ = 3.14159;
};

#endif