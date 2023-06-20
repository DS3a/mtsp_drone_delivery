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
        const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
        double x = se2_state->getX();
        double y = se2_state->getY();
        // this is the x and y of the sampled state, we are ignoring yaw for now

        std::shared_lock<std::shared_mutex> read_lock = this->swarm_config_tracker_->read_swarm_config();

        // TODO get the current drone's state
        // find the footprint extension factor based on the distance to this point
        bool collision = false;

        Eigen::Vector4d current_drone_state = (*this->swarm_config_tracker_->drone_states_)[this->drone_index_];
        double dist_to_sampled_point = calculate_distance(x, y, current_drone_state[0], current_drone_state[1]);
        double drone_speed = calculate_distance(current_drone_state[2], 0, 0, current_drone_state[3]);
        // check function definition to clarify this, drone_state[2] is x_vel and drone_state[3] is y_vel

        double time_to_reach_sample = dist_to_sampled_point / drone_speed;

        for (int i=0; i < this->swarm_config_tracker_->drone_states_->size(); i++) {
            if (i == this->drone_index_) {
                continue;
            }
            Eigen::Vector4d drone_state = (*this->swarm_config_tracker_->drone_states_)[i];
            const double drone_x = drone_state[0];
            const double drone_y = drone_state[1];

            if (calculate_distance(x, y, drone_x, drone_y) < 0.3) {
                collision = true;
                return !collision;
            }
        }
        read_lock.unlock();

        return !collision;
    }
} // namespace swarm_planner

#endif // STATE_VALIDITY_CHECKER_H_
