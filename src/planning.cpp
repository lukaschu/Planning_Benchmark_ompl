#include "benchmark_planning/planning.hpp"
#include "benchmark_planning/utils.hpp"
//#include "benchmark_planning/collision_checker.hpp"

const std::string MOVE_GROUP = "ur_manipulator";
const double PROP_STEPSIZE = 0.25;
const double MAX_SOLVETIME = 60.0;

using MyDuration = std::chrono::duration<double>;

Planning::Planning()
: Node("planner"), move_group_interface_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{   
    // Debug
    debug_publisher_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("robot_trajectory", 10); // publish the plan
    // End Debug

    // initialize the collision checker (after intializing the robot model)
    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    collision_checker_ = std::make_shared<Collision_Checker>(kinematic_model);

    // planning space
    space_ = std::make_shared<ob::CompoundStateSpace>();

    // define elements that are contained within space
    auto position = std::make_shared<ob::RealVectorStateSpace>(6);
    auto velocity = std::make_shared<ob::RealVectorStateSpace>(6);
    auto time = std::make_shared<ob::TimeStateSpace>();

    space_->addSubspace(position, 0.75);
    space_->addSubspace(velocity, 0.25);
    space_->addSubspace(time, 0);  

    /*
    STATE IS DEFINED AS: (rad.or rad/s)
    pos1    vel1    time
    pos2    vel2
    pos3    vel3
    pos4    vel4
    pos5    vel5
    pos6    vel6
    */

    // Get state bounds, total of seven 6 joints (originate from: https://github.com/fmauch/Universal_Robots_ROS2_Description/blob/ros2/config/ur10e/joint_limits.yaml)
    ob::RealVectorBounds positionBounds(6);
    ob::RealVectorBounds velocityBounds(6);
    // 1.
    positionBounds.setLow(0, -180);
    positionBounds.setHigh(0, 180);
    velocityBounds.setLow(0, -180);
    velocityBounds.setHigh(0,180);
    // 2.
    positionBounds.setLow(1,-180);
    positionBounds.setHigh(1,180);
    velocityBounds.setLow(1, -180);
    velocityBounds.setHigh(1, 180);
    // 3.
    positionBounds.setLow(2, -180);
    positionBounds.setHigh(2, 180); // was -45
    velocityBounds.setLow(2, -180);
    velocityBounds.setHigh(2, 180);
    // 4.
    positionBounds.setLow(3, -180); // was -45
    positionBounds.setHigh(3, 180);
    velocityBounds.setLow(3, -180);
    velocityBounds.setHigh(3, 180);
    // 5.
    positionBounds.setLow(4, -180);
    positionBounds.setHigh(4, 180);
    velocityBounds.setLow(4, -180);
    velocityBounds.setHigh(4, 180);
    // 6.
    positionBounds.setLow(5, -180);
    positionBounds.setHigh(5, 180);
    velocityBounds.setLow(5, -180);
    velocityBounds.setHigh(5, 180);

    position->setBounds(positionBounds);
    velocity->setBounds(velocityBounds);

    space_->sanityChecks();

    // Define elements that are defined within control space
    ctrl_space_ = std::make_shared<oc::RealVectorControlSpace>(space_, 6);

    /*
    CONTROL SPACE IS DEFINED AS: (rad/s²)
    acc1
    acc2
    acc3
    acc4
    acc5
    acc6
    */
    
    //Get ctrl bounds (originate from: https://github.com/fmauch/Universal_Robots_ROS2_Description/blob/ros2/config/ur10e/joint_limits.yaml)
    ob::RealVectorBounds controlBounds(6);

    // 1. (max jerk = 360)
    controlBounds.setLow(0, -180);
    controlBounds.setHigh(0, 180);
    //2. (max jerk = 180)
    controlBounds.setLow(1, -90);
    controlBounds.setHigh(1, 90);
    //3. (max jerk = 180)
    controlBounds.setLow(2, -90);
    controlBounds.setHigh(2, 90);
    //4. (max jerk = 360)
    controlBounds.setLow(3, -180);
    controlBounds.setHigh(3, 180);
    //5. (max jerk = 360)
    controlBounds.setLow(4, -180);
    controlBounds.setHigh(4, 180);
    //6. (max jerk = 360)
    controlBounds.setLow(5, -180);
    controlBounds.setHigh(5, 180);

    ctrl_space_->setBounds(controlBounds);

    // Definition of space information
    si_ = std::make_shared<oc::SpaceInformation>(space_, ctrl_space_);

    // Define collsion checker
    si_->setStateValidityChecker([this](const ob::State *state) { return collision_checker_->is_state_valid(si_.get(), state); });


    // Define motion specific constraints
    //si_->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si_, 90))

    // Define dynamic propagation for the system
    si_->setStatePropagator(dynamics);
    si_->setPropagationStepSize(PROP_STEPSIZE); 
    si_->setMinMaxControlDuration(2,80);

    si_->setup();
}

/*
Function that establishes the double integrator dynamics 
*/
void Planning::dynamics(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    /*
    use forward Euler integrator by implementing:      x[n] = x[n-1] + dT * v[n-1] + 0.5 * dt^2 * a[n-1]
                                                       v[n] = v[n-1] + dT * a[n-1]
    system input is directly set on the acceleration:  a[n] = u[n]
    */


   // lambda function for normalization of propagated state
   auto normalize_angle = [](double angle) -> double {
        angle = fmod(angle + 180.0, 360.0);
        if (angle < 0)
            angle += 360.0;
        return angle - 180.0;
    };

    // past state
    const auto position = start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const auto velocity = start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values;
    const auto time = start->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(2)->position;

    const double *ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // time
    result->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(2)->position = time + duration;
    // velocity
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = velocity[0] + ctrl[0] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = velocity[1] + ctrl[1] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[2] = velocity[2] + ctrl[2] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[3] = velocity[3] + ctrl[3] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[4] = velocity[4] + ctrl[4] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[5] = velocity[5] + ctrl[5] * duration;
    // position q1
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = normalize_angle(position[0] + 
    velocity[0] * duration + 0.5 * ctrl[0] * std::pow(duration,2)); 
    // position q2
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = normalize_angle(position[1] + 
    velocity[1] * duration + 0.5 * ctrl[1] * std::pow(duration,2));
    // position q3
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = normalize_angle(position[2] + 
    velocity[2] * duration + 0.5 * ctrl[2] * std::pow(duration,2));
    // position q4
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[3] = normalize_angle(position[3] + 
    velocity[3] * duration + 0.5 * ctrl[3] * std::pow(duration,2));
    // position q5
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[4] = normalize_angle(position[4] + 
    velocity[4] * duration + 0.5 * ctrl[4] * std::pow(duration,2));
    // position q6
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[5] = normalize_angle(position[5] + 
    velocity[5] * duration + 0.5 * ctrl[5] * std::pow(duration,2));
}

/*
Function that gets the scenario and loads it in
Can subsequently be used for initialization of collision scene
*/
void Planning::call_scenario_loader(std::string scenario)
{   
    // loads the scenario (e.g loads in the meshes and the primitives that build up the scenario)
    collision_checker_->load_scenario(scenario);
}

/*
Function that assigns inital and final state to the scoped state pointer
*/
void Planning::set_initial(ob::ScopedState<> *initial,ob::ScopedState<> *final, std::vector<double> initial_state, std::vector<double> final_state)
{   
    for(unsigned int i = 0; i < 6; ++i)
    {   
        (*initial)[i] = initial_state[i];
        (*final)[i] = final_state[i];
        (*initial)[i + 6] = 0.0; // velocity 
        (*final)[i + 6] = 0.0; // velocity
    }
}

/*
In here we recover the right format for the path which is needed for the moveit.execute(path) function
*/
MoveGroupInterface::Plan Planning::recover_moveit_path(ob::PathPtr &path, double duration, ob::State *start)
{
    // Now we try to bring the solution path into the right format
    // We need a moveit::plan object with attributes:
    // 1) start_state of type moveit_msgs::msg::RobotState
    // 2) trajectory of type moveit_msgs::msg::Robot_trajectory
    // 3) planning_time of type double (time that it took us planning)

    // object that we return
    MoveGroupInterface::Plan plan; 

    // FIRST ARGUMENT
    plan.planning_time = duration; 

    // SECOND ARGUMENT
    moveit_msgs::msg::RobotState start_state;

    // names
    start_state.joint_state.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    auto compound_placeholder = start->as<ob::CompoundState>();
    // first cover the positions
    float pos0 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[0] / 180 * pi_;
    float pos1 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[1] / 180 * pi_;
    float pos2 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[2] / 180 * pi_;
    float pos3 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[3] / 180 * pi_;
    float pos4 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[4] / 180 * pi_;
    float pos5 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[5] / 180 * pi_;
    start_state.joint_state.position = {pos0, pos1, pos2, pos3, pos4, pos5};

    // second the velocity
    float vel0 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[0] / 180 * pi_;
    float vel1 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[1] / 180 * pi_;
    float vel2 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[2] / 180 * pi_;
    float vel3 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[3] / 180 * pi_;
    float vel4 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[4] / 180 * pi_;
    float vel5 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[5] / 180 * pi_;
    start_state.joint_state.velocity = {vel0, vel1, vel2, vel3, vel4, vel5};
    
    plan.start_state = start_state;
    
    // THIRD ARGUMENT
    moveit_msgs::msg::RobotTrajectory robot_trajectory; // is then passed to the plan
    trajectory_msgs::msg::JointTrajectory robot_joint_trajectory; // is an element needed for the robot_trajectory

    auto placeholder_path = path->as<oc::PathControl>(); // convert to appropriate type
    std::vector<ob::State*> &trajectory_path = placeholder_path->getStates(); // extract the trajectory
    std::vector<oc::Control*> &control_path = placeholder_path->getControls();// extract the controls

    path->print(std::cerr); // inspect the acquired solution

    for(long unsigned int i = 0; i < trajectory_path.size(); ++i)
    {   
        trajectory_msgs::msg::JointTrajectoryPoint point; // will contain the position, velocity and acceleration
        auto compound_placeholder = trajectory_path[i]->as<ob::CompoundState>(); // current value from the path (pos and vel)

        // first cover the positions
        float pos0 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[0] / 180 * pi_;
        float pos1 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[1] / 180 * pi_;
        float pos2 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[2] / 180 * pi_;
        float pos3 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[3] / 180 * pi_;
        float pos4 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[4] / 180 * pi_;
        float pos5 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(0)->values[5] / 180 * pi_;
        point.positions = {pos0, pos1, pos2, pos3, pos4, pos5};

        // second the velocity
        float vel0 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[0] / 180 * pi_;
        float vel1 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[1] / 180 * pi_;
        float vel2 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[2] / 180 * pi_;
        float vel3 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[3] / 180 * pi_;
        float vel4 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[4] / 180 * pi_;
        float vel5 = compound_placeholder->as<ob::RealVectorStateSpace::StateType>(1)->values[5] / 180 * pi_;
        point.velocities = {vel0, vel1, vel2, vel3, vel4, vel5};

        // Third the accelerations
        if(i != trajectory_path.size() - 1)
        {
            float acc0 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[0] / 180 * pi_;
            float acc1 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[1] / 180 * pi_;
            float acc2 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[2] / 180 * pi_;
            float acc3 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[3] / 180 * pi_;
            float acc4 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[4] / 180 * pi_;
            float acc5 = control_path[i]->as<oc::RealVectorControlSpace::ControlType>()->values[5] / 180 * pi_;
            point.accelerations = {acc0, acc1, acc2, acc3, acc4, acc5};
        }
        else
        {
            float acc0 = 0;
            float acc1 = 0;
            float acc2 = 0;
            float acc3 = 0;
            float acc4 = 0;
            float acc5 = 0;
            point.accelerations = {acc0, acc1, acc2, acc3, acc4, acc5};
        }

        double exec_time = compound_placeholder->as<ob::TimeStateSpace::StateType>(2)->position;

        //std::cerr<< "time to input : " << exec_time << std::endl;

        MyDuration result = MyDuration(exec_time);

        // Convert the duration to seconds
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(result).count();

        // Assign the converted value to point.time_from_start
        point.time_from_start.sec = static_cast<int32_t>(seconds);
        point.time_from_start.nanosec = static_cast<uint32_t>((seconds - static_cast<int32_t>(seconds)) * 1e9);

        // Finally add the point
        robot_joint_trajectory.points.push_back(point);
    }

    robot_joint_trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    robot_trajectory.joint_trajectory = robot_joint_trajectory;
    
    plan.trajectory = robot_trajectory;

    return plan;
}

/*
Function that solves the underlying problem with the desired method (choose between different smapling based methods)
*/
void Planning::solve(std::vector<double> initial_state, std::vector<double> final_state, ob::PathPtr &path)
{   
    // Defining initial and final state
    ob::ScopedState<> initial(space_);
    ob::ScopedState<> final(space_);

    // Here set_initial_final function 
    set_initial(&initial, &final, initial_state, final_state);

    // Problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si_);
    pdef->setStartAndGoalStates(initial, final, 6);

    // Defining the planner (EST, RRT, KPIECE, PDST) (is not very smooth but don't know how else)
    using PLANNER = oc::RRT;

    auto planner = std::make_shared<PLANNER>(si_);
    planner->setProblemDefinition(pdef);

    // Define a bias towards the goal
    planner->as<PLANNER>()->setGoalBias(0.5);
    
    // comment if RRT
    // space_->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(space_)));
    // planner->as<PLANNER>()->setProjectionEvaluator("myProjection");

    planner->setup();

    // Some Debug information on the problem 

    //si_->printSettings(std::cout);
    //pdef->print();

    // End Debug

    auto start = std::chrono::steady_clock::now(); // timer start

    ob::PlannerStatus solved = planner->ob::Planner::solve(MAX_SOLVETIME); // calling the ompl planner

    auto end = std::chrono::steady_clock::now(); // timer end
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); // duration btw start and end

    // First we get the solution path
    if(solved)
    {   
        std::cerr << "got solution in: " << duration.count() / 1000 << " seconds" << std::endl;
        path = pdef->getSolutionPath();

        // Debug for printing the path

        //std::stringstream pathStream;
        //path->print(pathStream);
        //std::cerr << path->getStates() << std::endl;

        // ENd DEBUG

        // We adjust the format from ob::PathPtr to MoveGroupInterface::Plan 
        MoveGroupInterface::Plan moveit_plan = recover_moveit_path(path, duration.count(), pdef->getStartState(0));

        // Debug
        debug_publisher_->publish(moveit_plan.trajectory);
        // End Debug

        // Execute the traj on the robot (and simultanously start the simulation TODO !!!!1
        move_group_interface_.asyncExecute(moveit_plan);
        collision_checker_->simulate_obstacles(); // moves the obstacles  
    }
    planner->clear(); // just clear it in the end
}

/*
Functionality that solves the planning problem using moveit
This part would need some modification to be used properly. It was used for debugging while
confining this codebase
*/
void Planning::solve_with_moveit()
{
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.35;
        msg.position.y = 0.4;
        msg.position.z = 1.2;
        return msg;
    }();

    // Apply target to move_group_interface
    
    move_group_interface_.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // DEBUG
    debug_publisher_->publish(plan.trajectory);
    // DONE DEBUG

    // Execute the plan
    if(success) {
        move_group_interface_.execute(plan);
    } else {
        std::cerr << "No success gadddaammnn" << std::endl;
    }
}

