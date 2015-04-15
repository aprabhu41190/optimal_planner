#include <ompl/base/OptimizationObjective.h>

using namespace ompl;

class SE3dis_OptimizationObjective : public ompl::base::OptimizationObjective
{
public:
    SE3dis_OptimizationObjective(const ompl::base::SpaceInformationPtr &si) : ompl::base::OptimizationObjective(si){}
    virtual ~SE3dis_OptimizationObjective(){}

    virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;

};
