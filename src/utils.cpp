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
    cellSizes_.resize(7);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;
    cellSizes_[2] = 0.1;
    cellSizes_[3] = 0.1;
    cellSizes_[4] = 0.1;
    cellSizes_[5] = 0.1;
}

void MyProjection::project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
{
    const double *values = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    projection(0) = values[0];
    projection(1) = values[1];
    projection(2) = values[2];
    projection(3) = values[3];
    projection(4) = values[4];
    projection(5) = values[5];
}