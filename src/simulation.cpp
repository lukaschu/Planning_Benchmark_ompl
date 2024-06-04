#include "benchmark_planning/simulation.hpp"
using std::placeholders::_1;

Simulation::Simulation()
: Node("simulator")
{   
    // Retrieve the scenario which is specified as a launch argument
    this->declare_parameter<std::string>("scenario", "1");

    // This function calls the obstacle_checker.define_scenario function which loads in all relevant obstacles and environments
    planner.call_scenario_loader(this->get_parameter("scenario").as_string()); // mostly for obstacles

    // callback function that saves the current state of the simulated environment 
    state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "joint_trajectory_controller/state",  1000, std::bind(&Simulation::get_current_state, this, _1));
}

/*
Runs the simulation with the given scenario and solver 
is only called as soon as the current state is recovered by the callback function: get_current_state
*/
void Simulation::run_simulation() 
{   
    // Get access to scenario
    std::string scenario = this->get_parameter("scenario").as_string();

    // Now we recover the goal
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmark_planning");
    std::string goal_scenario_path = package_share_directory + "/config/scenarios/scenario_" + scenario + "/goal.yaml";

    // Retrieve the goal configuration corresponding to the scenario
    std::filesystem::path ScenarioGoalPath{goal_scenario_path};
    YAML::Node GoalNode = YAML::LoadFile(ScenarioGoalPath);
    std::vector<double> goal_config = GoalNode["goal"].as<std::vector<double>>();

    RCLCPP_INFO(this->get_logger(), "Initializing the solver");

    planner.solve(initial_state_, goal_config, path);
    //planner.solve_with_moveit();
}

/*
callback function to recover the intial state and to start the simulation
*/
void Simulation::get_current_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{   
    auto msg_pos = msg->actual.positions;
    initial_state_.clear();

    for(unsigned int i = 0; i < 6; ++i)
    {
        initial_state_.push_back(msg_pos[i] / pi_ * 180); // should be in degrees
    }
    
    // If the sate is received, we start the simulation
    if(!state_received_)
    {
        run_simulation();
        state_received_ = true;
    }

}