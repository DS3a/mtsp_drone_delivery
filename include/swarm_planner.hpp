#ifndef PLANNING_H_
#define PLANNING_H_

#define DOUBLE_PRECISION

#include <shared_mutex>

#include "base.hpp"
#include "workspace.hpp"
#include "swarm_planner_deps/state_validity_checker.hpp"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"

namespace ob = ompl::base;
// namespace og = ompl::geometry;

namespace swarm_planner {
    class SwarmPlannerSE2 {
    private:
        std::shared_ptr<ob::SE2StateSpace> space;

        mutable std::shared_mutex drones_config_mut;
        std::shared_ptr<std::vector<Eigen::Vector4d>> drone_states_; // use this to get start
        std::shared_ptr<std::vector<Eigen::Vector2d>> drone_goals_;

        std::shared_ptr<std::vector<Eigen::Vector2d>> drone_paths;

    public:
        SwarmPlannerSE2(std::vector<Eigen::Vector2d> bounds);
        bool write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                    std::vector<Eigen::Vector2d> drone_goals);

        std::shared_lock<std::shared_mutex> read_drone_states();

        friend class SwarmStateValidityChecker;
    };

    SwarmPlannerSE2::SwarmPlannerSE2(std::vector<Eigen::Vector2d> workspace_bounds) {
        this->space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setHigh(0, workspace_bounds[0][0]);
        bounds.setLow(0, workspace_bounds[0][1]);
        bounds.setHigh(1, workspace_bounds[1][0]);
        bounds.setLow(1, workspace_bounds[1][1]);

        this->space->setBounds(bounds);
    }

    std::shared_lock<std::shared_mutex> SwarmPlannerSE2::read_drone_states() {
        return std::shared_lock<std::shared_mutex>(this->drones_config_mut);
    }

    bool SwarmPlannerSE2::write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                                 std::vector<Eigen::Vector2d> drone_goals) {
        std::unique_lock<std::shared_mutex> write_lock(this->drones_config_mut);
        if (drone_states.size() != drone_goals.size()) {
            return false;
        }

        *(this->drone_states_) = drone_states;
        *(this->drone_goals_) = drone_goals;

        return true;
    }
} // namespace swarm_planner

#endif // PLANNING_H_
