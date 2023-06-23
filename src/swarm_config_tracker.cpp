#include "swarm_planner_deps/swarm_config_tracker.hpp"

namespace swarm_planner {
    SwarmConfigTracker::SwarmConfigTracker() {
        this->drone_states_ = std::make_shared<std::vector<Eigen::Vector4d>>();
        this->drone_goals_ = std::make_shared<std::vector<Eigen::Vector2d>>();
    }

    bool SwarmConfigTracker::num_drones_is_set() {
        if (this->num_drones != -1) {
            return true;
        } else {
            std::cout << "num_drones is not set\n";
            return false;
        }
    }

    bool SwarmConfigTracker::set_num_drones(int drones) {
        if (drones > 0) {
            this->num_drones = drones;
            return true;
        } else {
            return false;
        }
    }

    bool SwarmConfigTracker::write_drone_states(std::vector<Eigen::Vector4d> drone_states) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_states.size()) {
                *(this->drone_states_) = drone_states;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_goals(std::vector<Eigen::Vector2d> drone_goals) {
        // TODO
    }

    bool SwarmConfigTracker::write_drone_active_vector(std::vector<bool> drone_active) {
        // TODO
    }

    bool SwarmConfigTracker::write_drone_radii(std::vector<double> drone_radii) {
        // TODO
    }

    bool SwarmConfigTracker::write_drone_capacities(std::vector<double> drone_capacities) {
        // TODO
    }

    bool SwarmConfigTracker::write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                                std::vector<Eigen::Vector2d> drone_goals) {
        if (this->num_drones == -1) {
            return false;
        }

        if (drone_states.size() != drone_goals.size()) {
            return false;
        }

        if (this->num_drones != drone_states.size()) {
            return false;
        }

        std::cout << "assigning values to drone_states_ and drone_goals_\n";

        if (this->drone_states_)
            this->drone_states_->clear();
        if (this->drone_goals_)
            this->drone_goals_->clear();

        // (*this->drone_states_) = drone_states;
        for (int i=0; i < drone_states.size(); i++) {
            this->drone_states_->push_back(drone_states[i]);
            this->drone_goals_->push_back(drone_goals[i]);
        }

        return true;
    }

    std::shared_lock<std::shared_mutex> SwarmConfigTracker::read_swarm_config() const {
        return std::shared_lock<std::shared_mutex>(this->swarm_config_mut);
    }
} // namespace swarm_planner
