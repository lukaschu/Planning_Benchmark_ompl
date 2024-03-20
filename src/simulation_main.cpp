#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "benchmark_planning/simulation.hpp"


int main(int argc, char ** argv)
{   
    // Ros functionality
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulation>();
    std::cerr << "Sim initialized" << std::endl;
    node->run_simulation();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    
    return 0;
}