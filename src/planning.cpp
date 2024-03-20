#include "benchmark_planning/planning.hpp"
#include "benchmark_planning/utils.hpp"

Planning::Planning()
: Node("planner")
{   
    // Get params
    this->declare_parameter("solver", rclcpp::PARAMETER_STRING);
    get_parameter("solver", solver_); 

    space_ = std::make_shared<ob::CompoundStateSpace>();

    // define elements that are contained within space
    auto position = std::make_shared<ob::RealVectorStateSpace>(6);
    auto velocity = std::make_shared<ob::RealVectorStateSpace>(6);
    auto time = std::make_shared<ob::TimeStateSpace>();

    space_->addSubspace(position, 0.8);
    space_->addSubspace(velocity, 0.2);
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
    positionBounds.setLow(0, -190);
    positionBounds.setHigh(0, -35);
    velocityBounds.setLow(0, -90);
    velocityBounds.setHigh(0,90);
    // 2.
    positionBounds.setLow(1,-150);
    positionBounds.setHigh(1,-90);
    velocityBounds.setLow(1, -45);
    velocityBounds.setHigh(1, 45);
    // 3.
    positionBounds.setLow(2, -160);
    positionBounds.setHigh(2, -45);
    velocityBounds.setLow(2, -45);
    velocityBounds.setHigh(2, 45);
    // 4.
    positionBounds.setLow(3, -45);
    positionBounds.setHigh(3, 90);
    velocityBounds.setLow(3, -90);
    velocityBounds.setHigh(3, 90);
    // 5.
    positionBounds.setLow(4, 0);
    positionBounds.setHigh(4, 180);
    velocityBounds.setLow(4, -90);
    velocityBounds.setHigh(4, 90);
    // 6.
    positionBounds.setLow(5, -180);
    positionBounds.setHigh(5, 180);
    velocityBounds.setLow(5, -90);
    velocityBounds.setHigh(5, 90);

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
    si_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(si_.get(), state); });


    // Define motion specific constraints
    //si_->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si_, 90))

    // Define dynamic propagation for the system
    si_->setStatePropagator(dynamics);
    si_->setPropagationStepSize(0.1); 
    si_->setMinMaxControlDuration(2,30);

    si_->setup();
}

/*
Function that checks if a state given as (q,q_dot,t) /in R^7, is a valid state  
*/
bool Planning::isStateValid(const oc::SpaceInformation *si, const ob::State *state) 
{   
    if (state == nullptr)
    {
        return false;
    }
    else
    {
        return true;
    }
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
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[3] = velocity[3] + ctrl[0] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[4] = velocity[4] + ctrl[1] * duration;
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[5] = velocity[5] + ctrl[2] * duration;
    // position q1
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = position[0] + 
    velocity[0] * duration + 0.5 * ctrl[0] * std::pow(duration,2); 
    // position q2
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = position[1] + 
    velocity[1] * duration + 0.5 * ctrl[1] * std::pow(duration,2);
    // position q3
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = position[2] + 
    velocity[2] * duration + 0.5 * ctrl[2] * std::pow(duration,2);
    // position q4
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[3] = position[3] + 
    velocity[3] * duration + 0.5 * ctrl[3] * std::pow(duration,2);
    // position q5
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[4] = position[4] + 
    velocity[4] * duration + 0.5 * ctrl[4] * std::pow(duration,2);
    // position q6
    result->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[5] = position[5] + 
    velocity[5] * duration + 0.5 * ctrl[5] * std::pow(duration,2);

    return;
}

/*
Function that gets the scenario and loads it in
Can subsequently be used for collision checking
*/
void Planning::load_scenario()
{
    
}

/*
Function that shares the intitial and final condition
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
Function that solves the underlying problem with the desired method (choose between different smapling based methods)

NOTE: -Maybe pass initial and final state differently (as geometric states)
      -Need to include moving goal
*/
void Planning::solve(std::vector<double> initial_state, std::vector<double> final_state, ob::PathPtr &path)
{   
    // Defining initial and final state
    ob::ScopedState<> initial(space_);
    ob::ScopedState<> final(space_);

    // Here set_initial_final function (at moment is random)
    set_initial(&initial, &final, initial_state, final_state);

    // Problem definition
    auto pdef = std::make_shared<ob::ProblemDefinition>(si_);
    pdef->setStartAndGoalStates(initial, final);

    // Defining the planner (EST, RRT, KPIECE, PDST) (is not very smooth but don't know how else)
    using PLANNER = oc::RRT;

    auto planner = std::make_shared<PLANNER>(si_);
    planner->setProblemDefinition(pdef);

    // Define a bias towards the goal
    planner->as<PLANNER>()->setGoalBias(0.1);
    
    // Uncomment if RRT
    //space_->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(space_)));
    //planner->as<PLANNER>()->setProjectionEvaluator("myProjection");

    planner->setup();

    std::cerr << "solving the problem" <<std::endl;

    //si_->printSettings(std::cout);
    //pdef->print();

    auto start = std::chrono::steady_clock::now(); // timer start
    // Solve the problem (max t seconds)
    ob::PlannerStatus solved = planner->ob::Planner::solve(500.0);
    auto end = std::chrono::steady_clock::now(); // timer end
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if(solved)
    {   
        std::cerr << "got solution in: " << duration.count() / 1000 << " seconds" << std::endl;
        path = pdef->getSolutionPath();
        path->print(std::cerr);
    }
    planner->clear();
}
