#ifndef SWARM_PLANNER_H_
#define SWARM_PLANNER_H_

#define DOUBLE_PRECISION

#include <Eigen/Dense>
#include <Eigen/Core>
#include <shared_mutex>
#include <thread>
#include <future>

// #include "base.hpp"
// #include "workspace.hpp"
#include "swarm_planner_deps/state_validity_checker.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/Path.h"

#include "ompl/geometric/planners/fmt/FMT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/SORRTstar.h"
#include "ompl/geometric/planners/est/EST.h"

#include "ompl/geometric/PathGeometric.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// using current_planner = og::FMT;
using current_planner = og::InformedRRTstar;
// using current_planner = og::SORRTstar;
// using current_planner = og::RRTConnect;
// using current_planner = og::EST;

namespace swarm_planner {
    class SwarmPlannerSE2 {
    private:
        std::shared_ptr<ob::SE2StateSpace> space;
        std::vector<std::shared_ptr<ob::SpaceInformation>> si_vector;
        std::vector<std::shared_ptr<SwarmStateValidityChecker>> state_validity_checker_vector;
        std::vector<std::shared_ptr<current_planner>> planner_vector;

        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
        std::shared_ptr<std::vector<std::vector<Eigen::Vector2d>>> drone_paths;
        std::shared_ptr<std::vector<bool>> drones_path_found;

        void initialize_planners();

    public:
        std::tuple<std::vector<bool>, std::vector<std::vector<Eigen::Vector2d>>> get_paths();
        SwarmPlannerSE2(std::vector<Eigen::Vector2d> bounds, std::shared_ptr<SwarmConfigTracker> swarm_config_tracker);
        bool plan_paths();
        bool write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                    std::vector<Eigen::Vector2d> drone_goals);
    };

/*
    bool SwarmPlannerSE2::write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                                 std::vector<Eigen::Vector2d> drone_goals) {
        std::cout << "received new states and goals\n";
        bool valid_config = this->swarm_config_tracker_->write_swarm_config(drone_states, drone_goals);
        std::cout << "written states and goals to config tracker\n";
        if (valid_config) {
            std::cout << "received a valid configuration, now planning the paths\n";
            this->drone_paths->resize(drone_states.size());
            this->drones_path_found->resize(drone_states.size());

            // std::cout << "resized the drone paths\n";
            std::vector<std::vector<Eigen::Vector2d>> temp_paths(drone_states.size());
            std::vector<bool> temp_path_founds(drone_states.size(), false);

            std::vector<std::jthread> planning_threads;
            for (int i = 0; i < drone_states.size(); i++) {
                Eigen::Vector4d temp_drone_state = drone_states[i];
                Eigen::Vector2d temp_drone_goal = drone_goals[i];
                planning_threads.push_back(std::jthread([&temp_paths, &temp_path_founds, i, temp_drone_state, temp_drone_goal, this]() {
                    auto planner(std::make_shared<current_planner>(this->si));

                    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si));
                    std::cout << "initialized pdef in thread " << i << std::endl;

                    ob::ScopedState<> start(this->space);
                    ob::ScopedState<> goal(this->space);
                    std::cout << "initializing start and goal\n";

                    start->as<ompl::base::SE2StateSpace::StateType>()->setXY(temp_drone_state[0], temp_drone_state[1]);
                    start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);

                    goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(temp_drone_goal[0], temp_drone_goal[1]);
                    goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);

                    pdef->setStartAndGoalStates(start, goal);

                    planner->setProblemDefinition(pdef);
                    planner->setup();

                    std::cout << "attempting to solve for a path for drone " << i << std::endl;
                    // std::cout << "the start point is " << start << std::endl;
                    ob::PlannerStatus solved = planner->ob::Planner::solve(0.015);
                    std::cout << "attempt complete for drone " << i << std::endl;
                    if (solved) {
                        std::cout << "path for drone " << i << " found\n";
                        temp_path_founds[i] = true;
                        ob::PathPtr base_path = pdef->getSolutionPath();

                        og::PathGeometric path( dynamic_cast< const og::PathGeometric& >( *base_path ));

                        unsigned int path_len = path.getStateCount();
                        std::vector<ob::State*> states = path.getStates();
                        temp_paths[i].clear();
                        for (int j=0; j < path_len; j++) {
                            const ob::State* state = states[j];
                            const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
                            double x = se2_state->getX();
                            double y = se2_state->getY();
                            std::cout << "Drone " << i << " path idx " << j << ": (" << x << ", " << y << ")\n";
                            temp_paths[i].push_back(Eigen::Vector2d(x, y));
                        }
                    }
              }));
            }

            for (int i=0; i<planning_threads.size(); i++) {
                planning_threads[i].join();
                *(this->drones_path_found) = temp_path_founds;
                *(this->drone_paths) = temp_paths;
            }
        } else {
            std::cout << "the configuration is not valid\n";
        }

        return valid_config;
    }
*/
} // namespace swarm_planner

#endif // SWARM_PLANNER_H_
