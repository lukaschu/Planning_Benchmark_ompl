#include "benchmark_planning/simulation.hpp"

Simulation::Simulation()
: Node("simulator")
{   
    // Need to define the obstacle configs. then load them in in load scenario
    planner.load_scenario(); // mostly for obstacles
}

/*
Runs the simulation with the given scenario and solver (maybe call from main function or define a subscriber/service/action )
*/
void Simulation::run_simulation() 
{
    //planner.solve(initial, final, path)
    std::vector<double> initial_state(6);
    std::vector<double> final_state(6);

    initial_state[0] = -35.0;
    initial_state[1] = -90.0;
    initial_state[2] = -45.0;
    initial_state[3] = 90.0;
    initial_state[4] = 180.0;
    initial_state[5] = 180.0;

    final_state[0] = -45.0;
    final_state[1] = -100.0;
    final_state[2] = -52.0;
    final_state[3] = 80.0;
    final_state[4] = 165.0;
    final_state[5] = 165.0;

    planner.solve(initial_state, final_state, path);
}