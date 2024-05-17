#ifndef COLLISION_CHECKER
#define COLLISION_CHECKER

#include <iostream>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class Collision_Checker
{
    public:
        Collision_Checker(const moveit::core::RobotModelPtr& kinematic_model);
        bool is_state_valid(const oc::SpaceInformation *si, const ob::State *state);
        void load_scenario(double t); // updating scene for planning
        void update_scenario(); // updating scene for simulation
    private:
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; // for visualization
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_; // for collision checking
        const double pi_ = 3.14159;
};

#endif
