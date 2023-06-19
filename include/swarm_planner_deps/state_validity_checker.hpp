#ifndef STATE_VALIDITY_CHECKER_H_
#define STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <cmath>

namespace ob = ompl::base;

double calculate_distance(double x1, double y1, double x2, double y2) {
    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    return distance;
}

namespace swarm_planner {
    class SwarmStateValidityChecker: public ob::StateValidityChecker {
    private:
        int drone_index_;
        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
    public:
        SwarmStateValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si) {}

        virtual bool isValid(const ob::State *state) const override;

        void set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker);
        void set_drone_index(int drone_index);
    };

    void SwarmStateValidityChecker::set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker) {
        this->swarm_config_tracker_ = swarm_config_tracker;
    }

    void SwarmStateValidityChecker::set_drone_index(int drone_index) {
        this->drone_index_ = drone_index;
    }

    // TODO
    bool SwarmStateValidityChecker::isValid(const ob::State *state) const {
        return true;
        const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
        double x = se2_state->getX();
        double y = se2_state->getY();
        std::shared_lock<std::shared_mutex> read_lock = this->swarm_config_tracker_->read_swarm_config();
        bool collision = false;

        for (int i=0; i < this->swarm_config_tracker_->drone_states_->size(); i++) {
            Eigen::Vector4d drone_state = (*this->swarm_config_tracker_->drone_states_)[i];
            const double drone_x = drone_state[0];
            const double drone_y = drone_state[1];

            if (std::sqrt((x-drone_x)*(x-drone_x) + (y-drone_y)*(y-drone_y)) < 0.2) {
                collision = true;
                return !collision;
            }
        }
        read_lock.unlock();

        return !collision;
    }
} // namespace swarm_planner

#endif // STATE_VALIDITY_CHECKER_H_
