#ifndef COLLISION_CHECKER
#define COLLISION_CHECKER

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

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
        Collision_Checker(const moveit::core::RobotModelPtr& kinematic_model);
        bool is_state_valid(const oc::SpaceInformation *si, const ob::State *state);
        void load_scenario(); // load in the configs that define the whole scenario (executed once in the beginning)
        void load_scene(double t); // updating scene for planning (executed each time a new sample needs to be checked)
        void update_scene(double t); // updating scene for simulation
        void simulate_obstacles();
    private:
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; // for visualization
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_; // for collision checking
        // Each element at an index correpsonds to one and another in (msg_mesh_, trajectory_frame_, trajectory_, collision_object_timeinterval_)
        std::vector<std::shared_ptr<shape_msgs::msg::Mesh>> msg_mesh_;
        std::vector<std::shared_ptr<std::string>> trajectoryFrame_;
        std::vector<float> collision_object_timeinterval_; 
        // Each element of trajectory_ contains a ptr to to the full trajectory of a collision object
        // Is a tedious format but I wanted to keep it consistent
        std::vector<std::shared_ptr<std::vector<std::array<double, 7>>>> trajectory_; 
        const double pi_ = 3.14159;
};

#endif
