#ifndef SWARM_CONFIG_TRACKER_H_
#define SWARM_CONFIG_TRACKER_H_

#include <memory>
#include <shared_mutex>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>


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
} // namespace swarm_planner

#endif // SWARM_CONFIG_TRACKER_H_
