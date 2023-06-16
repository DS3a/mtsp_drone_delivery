#ifndef STATE_VALIDITY_CHECKER_H_
#define STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"

namespace ob = ompl::base;

namespace swarm_planner {
    class SwarmStateValidityChecker: public ob::StateValidityChecker {
    private:
    public:
      SwarmStateValidityChecker(const ob::SpaceInformationPtr &si)
          : ob::StateValidityChecker(si) {}
      virtual bool isValid(const ob::State *state) const override;

      void set_drone_states(
          const std::shared_ptr<std::vector<Eigen::Vector4d>> &drone_states);
    };
} // namespace swarm_planner

#endif // STATE_VALIDITY_CHECKER_H_
