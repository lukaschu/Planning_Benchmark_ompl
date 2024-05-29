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
        this->load_scene(current_time);

        moveit::core::RobotState& sampled_state = planning_scene_->getCurrentStateNonConst();
        sampled_state.setVariablePositions(sample_vec); // we pass it the current state

        // Do collision checking with moveit
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_result.clear();
        planning_scene_->checkCollision(collision_request, collision_result, sampled_state);

        if(collision_result.collision) 
        {
            return false; // we have collision
        }
        else
        {
            return true; // collision free
        }
    }
    else
    {   
        return false; // out of bounds
    }
}

/*
Loads in the objects contained within the scene at time t. Gets called everytime we do collision checking at time t
*/
void Collision_Checker::load_scene(double t)
{   
    // first remove all collision objects 
    planning_scene_->removeAllCollisionObjects();

    // iterate through all collision obejects that were loaded in load_scenario
    for(unsigned int i = 0; i < trajectoryFrame_.size(); ++i)
    {
        // define a new moveit collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = std::to_string(i);
        collision_object.header.frame_id = *trajectoryFrame_[i]; // adding the frame 

        // define the mesh that describes the collision object
        collision_object.meshes.resize(1);
        collision_object.meshes[0] = *msg_mesh_[i];

        // get the current pose of the object (according to the time t)
        std::vector<std::array<double, 7>> obstacles_trajectory = *trajectory_[i];
        int traj_index = 0;

        if(collision_object_timeinterval_[i] == 0)
        {
            continue; // static obstacle
        }
        else
        {
            traj_index = static_cast<int>(std::round(t / collision_object_timeinterval_[i])); // dynamic obstacle
        }

        auto pose{obstacles_trajectory[traj_index]};

        // set the pose to the collision object
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose[0];
        pose_msg.position.y = pose[1];
        pose_msg.position.z = pose[2];

        tf2::Quaternion quat_msg;

        quat_msg.setRPY(pose[3], pose[4], pose[5]);

        pose_msg.orientation.w = quat_msg.getW();   
        pose_msg.orientation.x = quat_msg.getX();
        pose_msg.orientation.y = quat_msg.getY();
        pose_msg.orientation.z = quat_msg.getZ();

        collision_object.pose = pose_msg; // pass the pose

        collision_object.operation = collision_object.ADD; // ADD means new object

        planning_scene_->processCollisionObjectMsg(collision_object);

        // At time 0 we also add the collision object to the scene in rviz
        if(t == 0)
        {   
            std::cerr << " adding the mesh" << std::endl;
            planning_scene_interface_->applyCollisionObject(collision_object);
        }
    }
}

/*
Executed once to get all objects 
This function saves all obstacles in a dynamic vector which can then be used in the load_Scene function to update the 
planning_scene
*/
void Collision_Checker::load_scenario()
{   
    // Retrieve the package share directory path
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmark_planning");

    // Construct the full path to the mesh file using package URI scheme
    std::string mesh_path = "package://benchmark_planning/config/obstacles/obstacle_00/mesh.ply";
    // Normal path to the trajectory information
    std::string trajectory_path =  package_share_directory + "/config/obstacles/obstacle_00/trajectory.yaml"; 

    // FIRST THE MESH

    // This part loads in the mesh and saves it to the member vector msg_mesh_
    // I dont specifically understand the details of each line 
    shapes::ShapeMsg mesh_msg;
    shapes::Mesh *mesh = shapes::createMeshFromResource(mesh_path); // maybe with .get()
    shapes::constructMsgFromShape(mesh, mesh_msg);
    msg_mesh_.push_back(std::make_shared<shape_msgs::msg::Mesh>(boost::get<shape_msgs::msg::Mesh>(mesh_msg)));

    // THEN THE TRAJECTORY AND THE FRAME

    // Retrieve the trajectory for the dynamic obstacles
    std::filesystem::path trajectoryPath{trajectory_path};

    YAML::Node trajectoryNode = YAML::LoadFile(trajectoryPath);
    std::string frame_placeholder = trajectoryNode["frame"].as<std::string>();
    std::vector<std::array<double, 7>> trajectory_placeholder = trajectoryNode["trajectory"].as<std::vector<std::array<double, 7>>>();

    trajectoryFrame_.push_back(std::make_shared<std::string>(frame_placeholder));
    trajectory_.push_back(std::make_shared<std::vector<std::array<double, 7>>>(trajectory_placeholder));

    // as a last step we recover the timestep which lies in between poses of the obstacles
    // if the obstacle is static we set it to zero!!
    if(trajectory_placeholder.size() > 1)
    {
        collision_object_timeinterval_.push_back(trajectory_placeholder[1][6]); // dynamic obstacle
    }
    else 
    {
        collision_object_timeinterval_.push_back(0); // static obstacle
    }
}

// Sets a timer and runs the simulation in real time
void Collision_Checker::simulate_obstacles()
{   
    // std::cerr << "simulating the objects now" << std::endl;

    // // We start by updating the scene every 0.1 seconds and simulate for 200 timesteps,
    // auto start = std::chrono::high_resolution_clock::now();
    
    // for(unsigned int i = 0; i <= 100; ++i)
    // {
    //     //auto update_start_time = std::chrono::high_resolution_clock::now();
    //     update_scene(0.2 * i); 
    //     //auto update_end_time = std::chrono::high_resolution_clock::now();
    //     //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(update_start_time - update_end_time);
    //     //std::cerr << "Duration " << duration.count() << std::endl;
    //     //std::this_thread::sleep_for(std::chrono::milliseconds(200) - duration);
    // }

    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    // //std::cerr << "start time is: " << start << " seconds" << std::endl;
    // std::cerr << "simulation took: " << duration.count() << " seconds" << std::endl;

}

/*
Move the obstacles in real time (for sim)
This function should be called simultaneously with the moveit.execute(function)
Idea is to call execute function that runs asynchronously
*/
void Collision_Checker::update_scene(double t) // time in seconds
{   
    // iterate through all collision obejects that were loaded in load_scenario
    for(unsigned int i = 0; i < trajectoryFrame_.size(); ++i)
    {   
        // define a new moveit collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = std::to_string(i);
        collision_object.header.frame_id = *trajectoryFrame_[i]; // adding the frame 
        
        // define the mesh that describes the collision object
        collision_object.meshes.resize(1);
        collision_object.meshes[0] = *msg_mesh_[i];

        // get the current pose of the object (according to the time t)
        std::vector<std::array<double, 7>> obstacles_trajectory = *trajectory_[i];
        int traj_index = 0;
        if(collision_object_timeinterval_[i] == 0)
        {
            continue; // static obstacle
        }
        else
        {
            traj_index = static_cast<int>(std::round(t / collision_object_timeinterval_[i])); // dynamic obstacle
        }

        auto pose{obstacles_trajectory[traj_index]};

        // set the pose to the collision object
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose[0];
        pose_msg.position.y = pose[1];
        pose_msg.position.z = pose[2];

        tf2::Quaternion quat_msg;

        quat_msg.setRPY(pose[3], pose[4], pose[5]);

        pose_msg.orientation.w = quat_msg.getW();   
        pose_msg.orientation.x = quat_msg.getX();
        pose_msg.orientation.y = quat_msg.getY();
        pose_msg.orientation.z = quat_msg.getZ();

        collision_object.pose = pose_msg; // pass the pose

        collision_object.operation = collision_object.ADD; // ADD means new object

        planning_scene_interface_->applyCollisionObject(collision_object); // Adds the new object and removes the old one automatically

    }
}