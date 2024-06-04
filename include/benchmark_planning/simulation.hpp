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
    void run_simulation(); // is called as soon the intial state is received and state_received_== true
    void get_current_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg); // callback function for subscription
private:
    Planning planner; // planning class 
    ob::PathPtr path; // will contain the relevant path
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_subscription_; // Subscriber to get the current state 
    std::vector<double> initial_state_; // will contain the infos for the planner on the initial state
    bool state_received_ = false; // a bool which given it is set to true, starts the simulation
    const double pi_ = 3.14159;
};

#endif