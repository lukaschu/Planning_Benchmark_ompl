/*
Helper class definitions
*/
#include "benchmark_planning/utils.hpp"

unsigned int MyProjection::getDimension(void) const
{
    return 6;
}

void MyProjection::defaultCellSizes(void)
{
    cellSizes_.resize(6);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;
    cellSizes_[2] = 0.1;
    cellSizes_[3] = 0.1;
    cellSizes_[4] = 0.1;
    cellSizes_[5] = 0.1;
}

void MyProjection::project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
{
    const double *values_pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double *values_vel = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values;

    // According to Fabian Zillenbiller, random projection is best
    projection(0) = values_pos[0];
    projection(1) = values_pos[1];
    projection(2) = values_pos[2];
    projection(3) = values_pos[3];
    projection(4) = values_pos[4];
    projection(5) = values_pos[5];
}

/*
projection(0) = 3 * values_pos[0] + 1.5 * values_vel[3];
    projection(1) = 1 * values_pos[1] + 4 * values_vel[0];
    projection(2) = 3.5 *values_pos[2] + 3 * values_vel[5];
    projection(3) = 2 * values_pos[3] + 2.5 * values_vel[4];
    projection(4) = 1.5 * values_pos[4] + 0.5 * values_vel[2];
    projection(5) = 4 * values_pos[5] + 2 * values_vel[1];
*/ 
