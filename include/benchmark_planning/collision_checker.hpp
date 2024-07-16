#ifndef COLLISION_CHECKER
#define COLLISION_CHECKER

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_bullet_plugin_loader.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include "yaml-cpp/yaml.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class Collision_Checker 
{
    public:
        Collision_Checker(const moveit::core::RobotModelPtr& kinematic_model,  rclcpp::Node *parent_node); 
        bool is_state_valid(const oc::SpaceInformation *si, const ob::State *state); // function passed to the ompl planner to check for collision
        void load_scenario(std::string scenario); // load in the configs that define the whole scenario (executed once in the beginning)
        void load_scene(double t); // updating scene for planning (executed each time a new sample needs to be checked)
        void update_scene(); // updating scene for simulation
        void simulate_obstacles();  // is called to cconfigure the simulation
    private:
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_; // for collision checking
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_; // publisher for visualization
        rclcpp::Node* parent_node_; // contains node in which this class is declared
        rclcpp::TimerBase::SharedPtr kalman_filter_timer_;
        double simulation_counter_ = 0;
        const double pi_ = 3.14159;

        /*

        DEFINITIONS, DESCRIBING THE COLLISION OBJECTS (EITHER PRIMITIVES OR MESHES)

        */

        // Each element at an index correpsonds to one and another in (msg_mesh_, mesh_trajectory_frame_, mesh_trajectory_, mesh_collision_object_timeinterval_)
        std::vector<std::shared_ptr<shape_msgs::msg::Mesh>> msg_mesh_;
        std::vector<std::shared_ptr<std::string>> mesh_trajectoryFrame_;
        std::vector<float> mesh_collision_object_timeinterval_; 
        // Each element of trajectory_ contains a ptr to to the full trajectory of a collision object
        // Is a tedious format but I wanted to keep it consistent
        std::vector<std::shared_ptr<std::vector<std::array<double, 7>>>> mesh_trajectory_; 

        // Each element at an index correpsonds to one and another in (msg_primitive_, primitive_trajectory_frame_, primitive_trajectory_, primitive_collision_object_timeinterval_)
        std::vector<std::shared_ptr<int>> msg_primitive_;
        std::vector<std::shared_ptr<std::string>> primitive_trajectoryFrame_;
        std::vector<float> primitive_collision_object_timeinterval_; 
        std::vector<std::shared_ptr<std::array<double, 3>>> primitive_dimensions_;
        // Each element of trajectory_ contains a ptr to to the full trajectory of a collision object
        // Is a tedious format but I wanted to keep it consistent
        std::vector<std::shared_ptr<std::vector<std::array<double, 7>>>> primitive_trajectory_; 
};

#endif
