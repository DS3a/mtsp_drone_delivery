#include "payload.hpp"
#include "swarm_planner.hpp"
#include "workspace.hpp"
#include "base.hpp"
#include <chrono>
#include <thread>

int main() {
    mtsp_drones_gym::Workspace ws(true);
    ws.add_drone(0, 0, 0.1, 1);
    ws.add_drone(-1.5, 0, 0.1, 1);
    ws.set_step_time(0.015);

    std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 1.5)};
    // std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 0)};


    mtsp_drones_gym::Move dronea, droneb;
    dronea = (mtsp_drones_gym::Move) {.x = 1, .y = 0};
    droneb = (mtsp_drones_gym::Move) {.x = 1, .y = 0};
    ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea, droneb});
    // ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea});

    std::vector<Eigen::Vector2d> workspace_dims = std::vector<Eigen::Vector2d>();
    workspace_dims.push_back(Eigen::Vector2d(2.25, -2.25));
    workspace_dims.push_back(Eigen::Vector2d(2, -2));

    swarm_planner::SwarmPlannerSE2 planner(workspace_dims);

    std::vector<Eigen::Vector4d> payload_states;

    for (int i=0; i<100; i++) {
        auto output = ws.step();
        std::vector<Eigen::Vector4d> drone_states = std::get<1>(output);
        std::cout << drone_states.size() << std::endl;
        planner.write_states_and_goals(drone_states, goals);
        std::vector<bool> paths_found;
        std::vector<std::vector<Eigen::Vector2d>> paths;

        std::tie(paths_found, paths) = planner.get_paths();
        ws.draw_paths(paths);


        std::this_thread::sleep_for(std::chrono::milliseconds(75));
    }

    return 0;
}
