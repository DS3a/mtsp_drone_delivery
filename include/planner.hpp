#ifndef PLANNING_H_
#define PLANNING_H_

#define DOUBLE_PRECISION
#include "base.hpp"
#include "swarm_planner/state_validity_checker.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometry;

namespace swarm_planner
class SwarmPlanner {
    private:
    auto space(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);

    public:
    SwarmPlanner
}


#endif // PLANNING_H_
