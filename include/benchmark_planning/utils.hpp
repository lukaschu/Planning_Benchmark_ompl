#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>

namespace ob = ompl::base;

class MyProjection : public ob::ProjectionEvaluator
{
public:
    MyProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space) {}

    virtual unsigned int getDimension(void) const;

    virtual void defaultCellSizes(void);

    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const;
};