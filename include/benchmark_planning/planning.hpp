#ifndef PLANNING
#define PLANNING

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>


//#include <ompl/base/RealVectorBounds.h>
//#include <ompl/base/CompoundStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class Planning : public rclcpp::Node
{
public:
    Planning();
    static bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);
    static void dynamics(const ob::State *start, const oc::Control *control, const double duration, ob::State *end);
    void load_scenario();
    void set_initial(ob::ScopedState<> *initial,ob::ScopedState<> *final, std::vector<double> initial_state, std::vector<double> final_state);
    void solve(std::vector<double> initial_state, std::vector<double> final_state, ob::PathPtr &path);

private:
    std::shared_ptr<ob::CompoundStateSpace> space_; // defines config. space (q,q_dot)
    std::shared_ptr<oc::RealVectorControlSpace> ctrl_space_; // defines input/ctrl space (acccel. = q_dot_dot)
    std::shared_ptr<oc::SpaceInformation> si_;
    std::string solver_; // Defines the sampling algo. that is used (RRT*, Kpiece ....)
};

#endif