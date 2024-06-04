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
Essentially, the collision scene is loaded into the planning scene
*/
void Collision_Checker::load_scene(double t)
{   
    // first remove all collision objects 
    planning_scene_->removeAllCollisionObjects();

    /*
    Mesh objects and trivial primitives are split up and considered sepperately in two loops
    */

    //  1) Iterate through all mesh collision obejects that were loaded in load_scenario
    for(unsigned int i = 0; i < mesh_trajectoryFrame_.size(); ++i)
    {
        // define a new moveit collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "mesh: " + std::to_string(i);
        collision_object.header.frame_id = *mesh_trajectoryFrame_[i]; // adding the frame 

        // define the mesh that describes the collision object
        collision_object.meshes.resize(1);
        collision_object.meshes[0] = *msg_mesh_[i];

        // get the current pose of the object (according to the time t)
        std::vector<std::array<double, 7>> obstacles_trajectory = *mesh_trajectory_[i];
        int traj_index = 0;

        if(!(mesh_collision_object_timeinterval_[i] == 0))
        {
            traj_index = static_cast<int>(std::round(t / mesh_collision_object_timeinterval_[i])); // dynamic obstacle
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

    // 2) Iterate through all primitive collision obejects that were loaded in load_scenario
    for(unsigned int i = 0; i < primitive_trajectoryFrame_.size(); ++i)
    {   
        // define a new moveit collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "primitive: " + std::to_string(i);
        collision_object.header.frame_id = *primitive_trajectoryFrame_[i]; // adding the frame

        // Define the primitive and add it as an obstacles
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = *msg_primitive_[i];
        primitive.dimensions.resize(3);

        std::array<double, 3> dimension_placeholder = *primitive_dimensions_[i];
        primitive.dimensions[0] = dimension_placeholder[0];
        primitive.dimensions[1] = dimension_placeholder[1];
        primitive.dimensions[2] = dimension_placeholder[2];

        // define the pose of the primitive
        collision_object.primitives.push_back(primitive);

        std::vector<std::array<double, 7>> primitive_obstacles_trajectory = *primitive_trajectory_[i];
        int traj_index = 0;

        if(!(primitive_collision_object_timeinterval_[i] == 0))
        {
            traj_index = static_cast<int>(std::round(t / mesh_collision_object_timeinterval_[i])); // dynamic obstacle
        }

        auto primitive_pose{primitive_obstacles_trajectory[traj_index]};

        geometry_msgs::msg::Pose primitive_pose_msg;
        primitive_pose_msg.position.x = primitive_pose[0];
        primitive_pose_msg.position.y = primitive_pose[1];
        primitive_pose_msg.position.z = primitive_pose[2];

        tf2::Quaternion primitive_quat_msg;

        primitive_quat_msg.setRPY(primitive_pose[3], primitive_pose[4], primitive_pose[5]);

        primitive_pose_msg.orientation.w = primitive_quat_msg.getW();   
        primitive_pose_msg.orientation.x = primitive_quat_msg.getX();
        primitive_pose_msg.orientation.y = primitive_quat_msg.getY();
        primitive_pose_msg.orientation.z = primitive_quat_msg.getZ();

        collision_object.pose = primitive_pose_msg; // pass the pose

        collision_object.operation = collision_object.ADD; // ADD means new object

        planning_scene_->processCollisionObjectMsg(collision_object);

        // At time 0 we also add the collision object to the scene in rviz
        if(t == 0)
        {   
            std::cerr << " adding the primtive" << std::endl;
            planning_scene_interface_->applyCollisionObject(collision_object);
        }

    }
}

/*
Executed once to get all objects 
This function saves all obstacles in a dynamic vector which can then be used in the load_Scene function to update the 
planning_scene
*/
void Collision_Checker::load_scenario(std::string scenario)
{   
    // Retrieve the package share directory path
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("benchmark_planning");
    std::filesystem::path Package_share_dir{package_share_directory};

    std::string scenario_path = "config/scenarios/scenario_" + scenario + "/obstacles";

    // will contain the paths fto either the meshes or primitives
    std::vector<std::string> primitive_paths;
    std::vector<std::string> mesh_paths;

    // According to the scenario arguemnt, we load in all obstacles which are either meshes or primitives
    for (auto const &dir_entry : std::filesystem::directory_iterator{std::filesystem::path(Package_share_dir / scenario_path)})
    {   
        //std::cerr << dir_entry.path() << std::endl;

        std::filesystem::path mesh_path_placeholder = dir_entry.path() / "mesh.ply";

        if(std::filesystem::exists(mesh_path_placeholder))
        {   
            mesh_paths.push_back(dir_entry.path());
        }
        else
        {   
            primitive_paths.push_back(dir_entry.path());
        }
    }

    // Helper lambda function to recover the uri path for loading the mesh (is being used below)
    auto convert_to_uri_path = [](const std::string& absolute_path, const std::string& uri_prefix) -> std::string {
        std::string uri_path = absolute_path;
        // Determine the search path dynamically
        size_t pos = absolute_path.find("/config/");
        if (pos != std::string::npos) {
            std::string search_path = absolute_path.substr(0, pos);
            // Replace the search path with the URI prefix
            uri_path.replace(0, search_path.length(), uri_prefix);
        }
        return uri_path;
        };

    // FIRST THE MESH
    for(unsigned int i = 0; i < mesh_paths.size(); ++i)
    {   
        // Need to convert the absolute path to an uri path for loading the mesh 
        std::string absolute_path = mesh_paths[i] + "/mesh.ply"; // placeholder
        std::string uri_prefix = "package://benchmark_planning"; //placeholder

        std::string mesh_path = convert_to_uri_path(absolute_path, uri_prefix); // mesh path
        std::string mesh_trajectory_path = mesh_paths[i] + "/trajectory.yaml"; // mesh traj. path

        // This part loads in the mesh and saves it to the member vector msg_mesh_
        // I dont specifically understand the details of each line 
        shapes::ShapeMsg mesh_msg;
        shapes::Mesh *mesh = shapes::createMeshFromResource(mesh_path); 
        shapes::constructMsgFromShape(mesh, mesh_msg);
        msg_mesh_.push_back(std::make_shared<shape_msgs::msg::Mesh>(boost::get<shape_msgs::msg::Mesh>(mesh_msg)));

        // THE  CORRESPONDING TRAJECTORY AND THE FRAME

        // Retrieve the trajectory for the dynamic obstacles
        std::filesystem::path MeshtrajectoryPath{mesh_trajectory_path};
        YAML::Node trajectoryNode_mesh = YAML::LoadFile(MeshtrajectoryPath);
        std::string frame_placeholder_mesh = trajectoryNode_mesh["frame"].as<std::string>();
        std::vector<std::array<double, 7>> trajectory_placeholder_mesh = trajectoryNode_mesh["trajectory"].as<std::vector<std::array<double, 7>>>();

        mesh_trajectoryFrame_.push_back(std::make_shared<std::string>(frame_placeholder_mesh));
        mesh_trajectory_.push_back(std::make_shared<std::vector<std::array<double, 7>>>(trajectory_placeholder_mesh));

        // as a last step we recover the timestep which lies in between poses of the obstacles
        // if the obstacle is static we set it to zero!!
        if(trajectory_placeholder_mesh.size() > 1)
        {
            mesh_collision_object_timeinterval_.push_back(trajectory_placeholder_mesh[1][6]); // dynamic obstacle
        }
        else 
        {
            mesh_collision_object_timeinterval_.push_back(0); // static obstacle
        }
    }

    // SECOND THE PRIMITIVE
    for(unsigned int i = 0; i < primitive_paths.size(); ++i)
    {   
        std::string primitive_path = primitive_paths[i] + "/shape.yaml";
        std::string primitive_trajectory_path =  primitive_paths[i] + "/trajectory.yaml";

        std::filesystem::path PrimitiveyPath{primitive_path};
        YAML::Node  primitiveNode = YAML::LoadFile(PrimitiveyPath);
        int primitive_type_placeholder = primitiveNode["type"].as<int>(); // this is the type of object we have
        msg_primitive_.push_back(std::make_shared<int>(primitive_type_placeholder));

        std::array<double, 3> primitive_dim_placeholder = primitiveNode["dimensions"].as<std::array<double, 3>>();
        primitive_dimensions_.push_back(std::make_shared<std::array<double, 3>>(primitive_dim_placeholder));

        // THE  CORRESPONDING TRAJECTORY AND THE FRAME

        // Retrieve the trajectory for the dynamic obstacles
        std::filesystem::path PrimitivetrajectoryPath{primitive_trajectory_path};
        YAML::Node trajectoryNode_primitive = YAML::LoadFile(PrimitivetrajectoryPath);
        std::string frame_placeholder_primitive = trajectoryNode_primitive["frame"].as<std::string>();
        std::vector<std::array<double, 7>> trajectory_placeholder_primitive = trajectoryNode_primitive["trajectory"].as<std::vector<std::array<double, 7>>>();

        primitive_trajectoryFrame_.push_back(std::make_shared<std::string>(frame_placeholder_primitive));
        primitive_trajectory_.push_back(std::make_shared<std::vector<std::array<double, 7>>>(trajectory_placeholder_primitive));

        if(trajectory_placeholder_primitive.size() > 1)
        {
            primitive_collision_object_timeinterval_.push_back(trajectory_placeholder_primitive[1][6]); // dynamic obstacle
        }
        else 
        {
            primitive_collision_object_timeinterval_.push_back(0); // static obstacle
        }
    }
}














// Problem is that the updating scene interface takes waaay too long


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
    // iterate through all collision mesh obejects that were loaded in load_scenario
    for(unsigned int i = 0; i < mesh_trajectoryFrame_.size(); ++i)
    {   
        // define a new moveit collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "mesh: " + std::to_string(i);
        collision_object.header.frame_id = *mesh_trajectoryFrame_[i]; // adding the frame 
        
        // define the mesh that describes the collision object
        collision_object.meshes.resize(1);
        collision_object.meshes[0] = *msg_mesh_[i];

        // get the current pose of the object (according to the time t)
        std::vector<std::array<double, 7>> obstacles_trajectory = *mesh_trajectory_[i];
        int traj_index = 0;
        if(mesh_collision_object_timeinterval_[i] == 0)
        {
            continue; // static obstacle
        }
        else
        {
            traj_index = static_cast<int>(std::round(t / mesh_collision_object_timeinterval_[i])); // dynamic obstacle
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