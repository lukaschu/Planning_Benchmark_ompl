#include "benchmark_planning/simulation.hpp"

Simulation::Simulation()
: Node("simulator")
{   
    // Need to define the obstacle configs. then load them in in load scenario
    planner.call_scenario_loader(); // mostly for obstacles
}

/*
Runs the simulation with the given scenario and solver (maybe call from main function or define a subscriber/service/action )
*/
void Simulation::run_simulation() 
{
    //planner.solve(initial, final, path)
    std::vector<double> initial_state(6);
    std::vector<double> final_state(6);

    initial_state[0] = 0.0;
    initial_state[1] = -90.0;
    initial_state[2] = 0.0;
    initial_state[3] = -90.0;
    initial_state[4] = 0.0;
    initial_state[5] = 0.0;

    final_state[0] = -20.0;
    final_state[1] = -100.0;
    final_state[2] = 20.0;
    final_state[3] = -75.0;
    final_state[4] = 10.0;
    final_state[5] = 10.0;

    RCLCPP_INFO(this->get_logger(), "Initializing the solver");

    planner.solve(initial_state, final_state, path);
    //planner.solve_with_moveit();
}