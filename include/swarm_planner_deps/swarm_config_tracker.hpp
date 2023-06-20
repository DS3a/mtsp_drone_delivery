#ifndef SWARM_CONFIG_TRACKER_H_
#define SWARM_CONFIG_TRACKER_H_

namespace swarm_planner {
    class SwarmConfigTracker {
    private:
        int num_drones = -1;
        mutable std::shared_mutex swarm_config_mut;
        std::shared_ptr<std::vector<Eigen::Vector4d>> drone_states_;
        std::shared_ptr<std::vector<Eigen::Vector2d>> drone_goals_;
        std::shared_ptr<std::vector<double>> drone_radii_;
        std::shared_ptr<std::vector<double>> drone_capacities_;
        std::shared_ptr<std::vector<bool>> drone_active_;

    public:
        SwarmConfigTracker();
        std::shared_lock<std::shared_mutex> read_swarm_config() const;
        bool write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                std::vector<Eigen::Vector2d> drone_goals);

        friend class SwarmPlannerSE2;
        friend class SwarmStateValidityChecker;
    };

    SwarmConfigTracker::SwarmConfigTracker() {
        this->drone_states_ = std::make_shared<std::vector<Eigen::Vector4d>>();
        this->drone_goals_ = std::make_shared<std::vector<Eigen::Vector2d>>();
    }

    bool SwarmConfigTracker::write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                                std::vector<Eigen::Vector2d> drone_goals) {
        if (drone_states.size() != drone_goals.size()) {
            return false;
        }

        this->num_drones = drone_states.size();
        std::cout << "assigning values to drone_states_ and drone_goals_\n";

        if (this->drone_states_)
            this->drone_states_->clear();
        if (this->drone_goals_)
            this->drone_goals_->clear();

        for (int i=0; i < drone_states.size(); i++) {
            this->drone_states_->push_back(drone_states[i]);
            this->drone_goals_->push_back(drone_goals[i]);
        }

        return true;
    }

    std::shared_lock<std::shared_mutex> SwarmConfigTracker::read_swarm_config() const {
        return std::shared_lock<std::shared_mutex>(this->swarm_config_mut);
    }
}


#endif // SWARM_CONFIG_TRACKER_H_
