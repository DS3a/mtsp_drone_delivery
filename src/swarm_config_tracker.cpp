#include "swarm_planner_deps/swarm_config_tracker.hpp"

namespace swarm_planner {
    SwarmConfigTracker::SwarmConfigTracker() {
        this->drone_states_ = std::make_shared<std::vector<Eigen::Vector4d>>();
        this->drone_goals_ = std::make_shared<std::vector<Eigen::Vector2d>>();
        this->drone_active_ = std::make_shared<std::vector<bool>>();
        this->drone_radii_ = std::make_shared<std::vector<double>>();
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
                for (int i=0; i < this->num_drones; i++) {
                    if ((*this->drone_active_)[i]) {
                        (*this->drone_states_)[i] = drone_states[i];
                    }
                }
                // (*this->drone_states_) = drone_states;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_goals(std::vector<Eigen::Vector2d> drone_goals) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_goals.size()) {
                std::cout << "drone goals updated\n";
                (*this->drone_goals_) = drone_goals;
                std::cout << (*this->drone_goals_)[0] << std::endl;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_active_vector(std::vector<bool> drone_active) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_active.size()) {
                (*this->drone_active_) = drone_active;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_radii(std::vector<double> drone_radii) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_radii.size()) {
                (*this->drone_radii_) = drone_radii;
                std::cout<<"radii";
                for(int i=0;i<this->num_drones;i++){
                    std::cout<<(*this->drone_radii_)[i]<<" ";
                }
                std::cout<<std::endl;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_capacities(std::vector<double> drone_capacities) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_capacities.size()) {
                (*this->drone_capacities_) = drone_capacities;
                return true;
            }
        }
        return false;
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

        (*this->drone_states_) = drone_states;
        (*this->drone_goals_) = drone_goals;

        return true;
    }

    std::shared_lock<std::shared_mutex> SwarmConfigTracker::read_swarm_config() const {
        return std::shared_lock<std::shared_mutex>(this->swarm_config_mut);
    }

    std::vector<Eigen::Vector4d> SwarmConfigTracker::read_drone_states() const {
        std::vector<Eigen::Vector4d> drone_states;
        {
            this->read_swarm_config();
            //creating a deep copy, chaging drone_states will not change drone_states_
            drone_states = *(this->drone_states_);
        }

        return drone_states;
    }

    std::vector<double> SwarmConfigTracker::read_drone_radii() const {
        std::vector<double> drone_radii;
        {
            this->read_swarm_config();
            // std::cout << "reading drone radii\n" << this->drone_radii_->size() << std::endl;
            drone_radii = *this->drone_radii_;
            // std::cout << drone_radii[0] << "dronee radiiii\n";
        }
        return drone_radii;
    }

    std::vector<bool> SwarmConfigTracker::read_drone_active() const {
        std::vector<bool> drone_active;
        {
            this->read_swarm_config();
            drone_active = *(this->drone_active_);
        }

        return drone_active;
    }

    std::vector<Eigen::Vector2d> SwarmConfigTracker::read_drone_goals() const {
        std::vector<Eigen::Vector2d> drone_goals;
        {
            this->read_swarm_config();
            std::cout << "the drone goals are\n";
            drone_goals = *(this->drone_goals_);
            for(int i=0; i < drone_goals.size(); i++) {
                std::cout << drone_goals[i] << " ";
            }
            std::cout << std::endl;
        }

        return drone_goals;
    }

    void SwarmConfigTracker::deactivate_drone(int drone_idx, Eigen::Vector2d destination) {
        (*this->drone_states_)[drone_idx] = Eigen::Vector4d(destination.x(), destination.y(), 0, 0);
        std::cout << "deactivate new drone position is " << (*this->drone_states_)[drone_idx];
    }
} // namespace swarm_planner
