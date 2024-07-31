#include "benchmark_planning/simulation.hpp"
using std::placeholders::_1;

Simulation::Simulation()
: Node("simulator")
{   
    // initialize planning class as shared ptr
    planner_ = std::make_shared<Planning>(this);

    // Retrieve the scenario which is specified as a launch argument
    this->declare_parameter<std::string>("scenario", "1");

    // callback function that saves the current state of the simulated environment 
    state_subscription_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "joint_trajectory_controller/state",  1000, std::bind(&Simulation::get_current_state, this, _1));

    // service caller which resets the gazebo world (/gazebo/reset_world, type: empty)
    gazebo_resetter = this->create_client<std_srvs::srv::Empty>("/reset_world");
}

Simulation::~Simulation()
{
    planner_.reset();
}

/*
Runs the simulation with the given scenario and solver 
is only called as soon as the current state is recovered by the callback function: get_current_state
*/
void Simulation::run_simulation(std::vector<double> true_start_state_pos) 
{   
    for(int i = 0; i <= 30; ++i)
    {
        // Get access to scenario
        std::string scenario = std::to_string(i);

        planner_->call_scenario_loader(scenario); // mostly for obstacles

        // Now we recover the goal
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmark_planning");
        std::string goal_scenario_path = package_share_directory + "/config/scenarios/final_scenario_" + scenario + "/goal.yaml";

        // Retrieve the goal configuration corresponding to the scenario
        std::filesystem::path ScenarioGoalPath{goal_scenario_path};
        YAML::Node GoalNode = YAML::LoadFile(ScenarioGoalPath);
        std::vector<double> goal_config = GoalNode["goal"].as<std::vector<double>>();

        RCLCPP_INFO(this->get_logger(), "Initializing the solver");

        planner_->solve(initial_state_, true_start_state_pos, goal_config, path);
        //planner_->solve_with_moveit();

        // Here we also need to reset the world and make include a sleep statement
        std::cerr << "We wait 10 seconds and then restart the planning algorithm" << std::endl; 

        rclcpp::sleep_for(std::chrono::seconds(10));

        while (!gazebo_resetter->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = gazebo_resetter->async_send_request(request);

        std::cerr << "We wait 5 seconds for the simulation to be reset" << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(5));
        
    }
}

/*
callback function to recover the intial state and to start the simulation
*/
void Simulation::get_current_state(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{   
    std::vector<double> true_start_state_pos = msg->actual.positions;
    initial_state_.clear();

    // Note that unfortunately the simulation returns values between -2pi and 2pi 
    for(unsigned int i = 0; i < 6; ++i)
    {   
        double squeezed_state = true_start_state_pos[i];

        if(squeezed_state > pi_)
        {
            //double squeezed_state = std::fmod(squeezed_state ,pi_);
            squeezed_state = squeezed_state  - 2 * pi_;
        }
        else if(squeezed_state < -pi_)
        {
            //double squeezed_state = std::fmod(squeezed_state ,pi_);
            squeezed_state = 2*pi_ + squeezed_state;
        }
        
        initial_state_.push_back(squeezed_state / pi_ * 180); // should be in degrees (between -180 to 180)
    }
    
    // If the sate is received, we start the simulation
    if(!state_received_)
    {
        run_simulation(true_start_state_pos);
        state_received_ = true;
    }

}