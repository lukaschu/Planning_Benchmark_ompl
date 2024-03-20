#ifndef SIMULATION
#define SIMULATION

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "benchmark_planning/planning.hpp"
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>

class Simulation : public rclcpp::Node
{   
public:
    Simulation();
    void run_simulation();
private:
    Planning planner;
    ob::PathPtr path;
    double intitial_1_;
    double intitial_2_;
    double intitial_3_;
    double intitial_4_;
    double intitial_5_;
    double intitial_6_;
    double final_1_;
    double final_2_;
    double final_3_;
    double final_4_;
    double final_5_;
    double final_6_;
    std::string solver_;
};

#endif