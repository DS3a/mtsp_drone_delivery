#ifndef SWARM_CONFIG_TRACKER_H_
#define SWARM_CONFIG_TRACKER_H_

namespace swarm_planner {
    class SwarmConfigTracker {
    private:
        mutable std::shared_mutex swarm_config_mut;
        std::shared_ptr<std::vector<Eigen::Vector4d>> drone_states_;
        std::shared_ptr<std::vector<Eigen::Vector2d>> drone_goals_;

    public:
        SwarmConfigTracker();
        std::shared_lock<std::shared_mutex> read_swarm_config() const;
        bool write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                std::vector<Eigen::Vector2d> drone_goals);
    };

    SwarmConfigTracker::SwarmConfigTracker() {}

    bool SwarmConfigTracker::write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                                std::vector<Eigen::Vector2d> drone_goals) {
        if (drone_states.size() != drone_goals.size()) {
            return false;
        }

        *(this->drone_states_) = drone_states;
        *(this->drone_goals_) = drone_goals;
    }

    std::shared_lock<std::shared_mutex> SwarmConfigTracker::read_swarm_config() const {
        return std::shared_lock<std::shared_mutex>(this->swarm_config_mut);
    }
}


#endif // SWARM_CONFIG_TRACKER_H_
