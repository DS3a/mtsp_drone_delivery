#ifndef PLANNING_H_
#define PLANNING_H_

#define DOUBLE_PRECISION

#include <shared_mutex>

#include "base.hpp"
#include "workspace.hpp"
#include "swarm_planner_deps/state_validity_checker.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"

namespace ob = ompl::base;
// namespace og = ompl::geometry;

namespace swarm_planner {
    class SwarmPlannerSE2 {
    private:
        std::shared_ptr<ob::SE2StateSpace> space;

        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;

        std::shared_ptr<std::vector<Eigen::Vector2d>> drone_paths;

        std::shared_ptr<SwarmStateValidityChecker> state_validity_checker_;

    public:
        SwarmPlannerSE2(std::vector<Eigen::Vector2d> bounds);
        bool write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                    std::vector<Eigen::Vector2d> drone_goals);
    };

    SwarmPlannerSE2::SwarmPlannerSE2(std::vector<Eigen::Vector2d> workspace_bounds) {
        this->space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setHigh(0, workspace_bounds[0][0]);
        bounds.setLow(0, workspace_bounds[0][1]);
        bounds.setHigh(1, workspace_bounds[1][0]);
        bounds.setLow(1, workspace_bounds[1][1]);

        this->space->setBounds(bounds);

        // this->state_validity_checker_ = std::make_shared<SwarmStateValidityChecker>(si);

    }

    bool SwarmPlannerSE2::write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                                 std::vector<Eigen::Vector2d> drone_goals) {
        return this->swarm_config_tracker_->write_swarm_config(drone_states, drone_goals);
    }
} // namespace swarm_planner

#endif // PLANNING_H_
