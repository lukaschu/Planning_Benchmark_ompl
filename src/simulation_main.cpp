#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "benchmark_planning/simulation.hpp"


int main(int argc, char ** argv)
{   
    // Ros functionality
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulation>();

    // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // executor->add_node(node->get_node_base_interface());
    // executor->spin();
    rclcpp::spin(node);
    //rclcpp::shutdown();
    
    return 0;
}