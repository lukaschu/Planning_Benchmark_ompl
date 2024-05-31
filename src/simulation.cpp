#include "benchmark_planning/simulation.hpp"
using std::placeholders::_1;

Simulation::Simulation()
: Node("simulator")
{   
    // Need to define the obstacle configs. then load them in in load scenario
    planner.call_scenario_loader(); // mostly for obstacles

    state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "joint_trajectory_controller/state",  1000, std::bind(&Simulation::get_current_state, this, _1));
}

/*
Runs the simulation with the given scenario and solver (maybe call from main function or define a subscriber/service/action )
*/
void Simulation::run_simulation() 
{   
    // Final state follows from scenario
    std::vector<double> final_state(6);
    final_state[0] = -20.0;
    final_state[1] = -100.0;
    final_state[2] = 20.0;
    final_state[3] = -75.0;
    final_state[4] = 10.0;
    final_state[5] = 10.0;

    RCLCPP_INFO(this->get_logger(), "Initializing the solver");

    planner.solve(initial_state_, final_state, path);
    //planner.solve_with_moveit();
}

/*
callback function to recover 
*/
void Simulation::get_current_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{   
    auto msg_pos = msg->actual.positions;
    initial_state_.clear();

    for(unsigned int i = 0; i < 6; ++i)
    {
        initial_state_.push_back(msg_pos[i] / pi_ * 180); // should be in degrees
    }
    
    if(!state_received_)
    {
        run_simulation();
        state_received_ = true;
    }

}