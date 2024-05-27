#include <benchmark_planning/collision_checker.hpp>

Collision_Checker::Collision_Checker(const moveit::core::RobotModelPtr& kinematic_model)
{
    // Define the variables needed
    // Initialized the scene interface (needed for visualization)
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
    planning_scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
}

/*
Function that checks if a state given as (q,q_dot,t) /in R^7, is a valid state  
*/
bool Collision_Checker::is_state_valid(const oc::SpaceInformation *si, const ob::State *state)
{
    // Check if state within bounds
    bool withinbounds = si->satisfiesBounds(state);
    if(withinbounds)
    {   
        // give current state the values from *state
        double sample1 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] / 180 * pi_;
        double sample2 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] / 180 * pi_;
        double sample3 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] / 180 * pi_;
        double sample4 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[3] / 180 * pi_;
        double sample5 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[4] / 180 * pi_;
        double sample6 = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[5] / 180 * pi_;

        std::vector<double> sample_vec{sample1, sample2, sample3, sample4, sample5, sample6};

        // extract the time to adjust the collision scene
        double current_time = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(2)->position;

        // We change the scenario such that it corresponds to the time correspponding to the state
        this->load_scenario(current_time);

        moveit::core::RobotState& sampled_state = planning_scene_->getCurrentStateNonConst();
        sampled_state.setVariablePositions(sample_vec); // we pass it the current state

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_result.clear();
        planning_scene_->checkCollision(collision_request, collision_result, sampled_state);

        if(collision_result.collision) // we have collision
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    else
    {   
        return false;
    }
}

/*
Loads in the static objects contained within the scene
*/
void Collision_Checker::load_scenario(double t)
{
    // first remove all objects 
    planning_scene_->removeAllCollisionObjects();

    // Define the objects
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.id = "box";

    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.id = "sphere";

    /* A default pose */
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = 0.0;
    pose1.position.y = 0.0;
    pose1.position.z = 1.85;
    pose1.orientation.w = 1.0;

    /* A default pose */
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = -1.0;
    pose2.position.y = 1.0;
    pose2.position.z = 0.0 + t*0.01;
    pose2.orientation.w = 1.0;

    /* Define a box to be attached */
    shape_msgs::msg::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.5;
    primitive1.dimensions[1] = 0.5;
    primitive1.dimensions[2] = 0.5;

    /* Define a box to be attached */
    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive2.SPHERE;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.5;
    primitive2.dimensions[1] = 0.5;
    primitive2.dimensions[2] = 0.5;

    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(pose1);

    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(pose2);

    // Defining add operation
    collision_object1.operation = collision_object1.ADD;

    // Defining add operation
    collision_object2.operation = collision_object2.ADD;

    // give th object a valid frame
    collision_object1.header.frame_id = "base_link";

    // give th object a valid frame
    collision_object2.header.frame_id = "base_link";

    // Fabians method (add object to planning_scene_ directly)
    planning_scene_->processCollisionObjectMsg(collision_object1);
    planning_scene_->processCollisionObjectMsg(collision_object2);

    if(t == 0)
    {   
        planning_scene_->printKnownObjects(std::cerr);
        // add object to interface for visualization (first we only apply it on the initial position)
        planning_scene_interface_->applyCollisionObject(collision_object1);
        planning_scene_interface_->applyCollisionObject(collision_object2);
    }

    // MOVE THE OBJECT (example)
    // Moves the obstacles relative to the previous position
    // collision_object2.operation = collision_object2.MOVE;
    // pose2.position.x = -1.0;
    // pose2.position.y = 1.0;
    // pose2.position.z = 0.0;
    // pose2.orientation.w = 1.0;

    // collision_object2.primitive_poses.push_back(pose2);
    // planning_scene_->processCollisionObjectMsg(collision_object2);
    // planning_scene_interface_->applyCollisionObject(collision_object2);

    // DEBUG 
    // std::vector<double> sample_vec{0, -1.57, 0, -1.57, 0, 0};

    // moveit::core::RobotState& sampled_state = planning_scene_->getCurrentStateNonConst();
    // sampled_state.setVariablePositions(sample_vec); // we pass it the current state

    // collision_detection::CollisionRequest collision_request;
    // collision_detection::CollisionResult collision_result;
    // collision_result.clear();
    // planning_scene_->checkCollision(collision_request, collision_result, sampled_state);

    // if(collision_result.collision)
    // {
    //     std::cerr<< "COLLISION" << std::endl;
    // }
    // else
    // {
    //     std::cerr<< "no collision" <<std::endl;
    // }
    // END DEBUG
}

void Collision_Checker::update_scenario()
{
    // Move the obstacles that are currently being moved around according to the timestamp (for sim)

}
