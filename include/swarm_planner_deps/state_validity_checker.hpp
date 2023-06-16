#ifndef STATE_VALIDITY_CHECKER_H_
#define STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

namespace ob = ompl::base;

namespace swarm_planner {
    class SwarmStateValidityChecker: public ob::StateValidityChecker {
    private:
        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
    public:
        SwarmStateValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si) {}

        virtual bool isValid(const ob::State *state) const override;

        void set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker);
    };

    void SwarmStateValidityChecker::set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker) {
        this->swarm_config_tracker_ = swarm_config_tracker;
    }

    bool SwarmStateValidityChecker::isValid(const ob::State *state) const {
        return true;
    }
} // namespace swarm_planner

#endif // STATE_VALIDITY_CHECKER_H_
