#ifndef PLANNING_H_
#define PLANNING_H_

#define DOUBLE_PRECISION

#include <shared_mutex>
#include <thread>
#include <future>

#include "base.hpp"
#include "workspace.hpp"
#include "swarm_planner_deps/state_validity_checker.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"
// #include "ompl/geometry/
#include "ompl/geometric/planners/fmt/FMT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace swarm_planner {
    class SwarmPlannerSE2 {
    private:
        std::shared_ptr<ob::SE2StateSpace> space;
        std::shared_ptr<ob::SpaceInformation> si;
        std::shared_ptr<SwarmStateValidityChecker> state_validity_checker_;

        std::shared_ptr<og::FMT> fmt_planner_;

        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
        std::shared_ptr<std::vector<std::vector<Eigen::Vector2d>>> drone_paths;

        void plan_path(std::promise<std::vector<Eigen::Vector2d>> && planned_path,
                       Eigen::Vector4d drone_state,
                       Eigen::Vector2d drone_goal);

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
        this->si = std::make_shared<ob::SpaceInformation>(this->space);

        this->state_validity_checker_ = std::make_shared<SwarmStateValidityChecker>(this->si);
        this->state_validity_checker_->set_swarm_config_tracker(this->swarm_config_tracker_);

        this->si->setStateValidityChecker(this->state_validity_checker_);
        this->fmt_planner_ = std::make_shared<og::FMT>(this->si);
    }

    void SwarmPlannerSE2::plan_path(std::promise<std::vector<Eigen::Vector2d>> && planned_path,
                                    Eigen::Vector4d drone_state,
                                    Eigen::Vector2d drone_goal) {
        auto pdef(std::make_shared<ob::ProblemDefinition>(this->si));
    }

    bool SwarmPlannerSE2::write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                                 std::vector<Eigen::Vector2d> drone_goals) {
        bool valid_config = this->swarm_config_tracker_->write_swarm_config(drone_states, drone_goals);
        if (valid_config) {
            this->drone_paths->resize(drone_states.size());
            std::vector<std::vector<Eigen::Vector2d>> temp_paths(drone_states.size());

            std::vector<std::jthread> planning_threads;
            for (int i = 0; i < drone_states.size(); i++) {
                Eigen::Vector4d temp_drone_state = drone_states[i];
                Eigen::Vector2d temp_drone_goal = drone_goals[i];
                planning_threads.push_back(std::jthread([&temp_paths, i, temp_drone_state, temp_drone_goal, this]() {
                    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si));
                    // TODO define problem statement, call planner, and modify temp_paths[i]
                    temp_paths[i] = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 1), Eigen::Vector2d(2, 2)};
              }));
            }

            for (int i=0; i<planning_threads.size(); i++) {
                planning_threads[i].join();
                *(this->drone_paths) = temp_paths;
            }
        }

        return valid_config;
    }
} // namespace swarm_planner

#endif // PLANNING_H_
