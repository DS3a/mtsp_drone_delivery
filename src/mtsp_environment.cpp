#include "payload.hpp"
#include "swarm_planner.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include "workspace.hpp"
#include "base.hpp"
#include <chrono>
#include <thread>
#include<vector>
#include "swarm_scheduler.hpp"

int main() {
    mtsp_drones_gym::Workspace ws(true);
    ws.add_drone(0, 0, 0.2, 1);
    ws.add_drone(0, 1, 0.1, 1);
    ws.add_drone(0, -1, 0.1, 1);
    ws.add_drone(-1, -1, 0.1, 1);
    ws.add_drone(-1.5, -1, 0.1, 1);
    ws.set_step_time(0.015);

    ws.add_payload(0, 1, 1, 2, 1);
    ws.add_payload(0,-1, 1,-2, 1);
    ws.add_payload(-1,2, 1, 1, 2);
    ws.add_payload(-1, 0, 1,-1,-1);
    ws.add_payload(-2,-1,1, 0,-1);
    std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(0, 0), Eigen::Vector2d(0, -0), Eigen::Vector2d(0, -0), Eigen::Vector2d(0, 0), Eigen::Vector2d(-0, 0)};
    // std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 0)};
    

    mtsp_drones_gym::Move dronea, droneb, dronec, droned, dronee;
    dronea = (mtsp_drones_gym::Move) {.x = 0.5, .y = 0.5};
    droneb = (mtsp_drones_gym::Move) {.x = 0, .y = -0};
    dronec = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    droned = (mtsp_drones_gym::Move) {.x = 1, .y = 0.5};
    dronee = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea, droneb, dronec, droned, dronee});
    // ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea});

    std::vector<Eigen::Vector2d> workspace_dims = std::vector<Eigen::Vector2d>();
    workspace_dims.push_back(Eigen::Vector2d(2.25, -2.25));
    workspace_dims.push_back(Eigen::Vector2d(2, -2));

    std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker = std::make_shared<swarm_planner::SwarmConfigTracker>();
    swarm_config_tracker->set_num_drones(5);

    swarm_config_tracker->write_swarm_config(std::vector<Eigen::Vector4d> {
        Eigen::Vector4d(0, 0, 0, 0),
        Eigen::Vector4d(0, 1, 0.1, 1),
        Eigen::Vector4d(0, -1, 0, 0),
        Eigen::Vector4d(1, -1, 0, 0),
        Eigen::Vector4d(-1.5, -1, 0, 0)
    }, goals);
    swarm_config_tracker->write_drone_active_vector(std::vector<bool>({true, true, true, true, true}));
    swarm_config_tracker->write_drone_radii(std::vector<double>({0.1, 0.1, 0.1, 0.1, 0.1}));

    swarm_planner::SwarmPlannerSE2 planner(workspace_dims, swarm_config_tracker);

    std::vector<Eigen::Vector4d> payload_states;

    ws.set_swarm_config_tracker(swarm_config_tracker);

    std::vector<std::vector<int>> mission_drones_list = {
        {0,0,1,0,0},
        {0,1,0,0,0},
        {0,0,0,1,0},
        {1,0,0,0,0},
        {0,0,0,0,1}
        //{0,1,0,0,1}
    };
    swarm_scheduler::SwarmScheduler sc;
    sc.intilization(mission_drones_list);
    sc.getpayload_data(ws.read_payloads());
    sc.set_swarm_config_tracker(swarm_config_tracker);

    for (int i=0; i<100; i++) {
        std::cout<<"in for loop"<<std::endl;
        sc.print_mission();
        sc.print_payloads();
        sc.mission_check();
        auto output = ws.step();
        std::vector<Eigen::Vector4d> drone_states = std::get<1>(output);
        swarm_config_tracker->write_swarm_config(drone_states, goals);
        // std::vector<double> v{i*0.01, i*0.01, i*0.01, i*0.01, i*0.01}; // vector with 100 ints.
        // std::iota (std::begin(v), std::end(v), 0.1); // Fill with 0, 1, ..., 99.

        // swarm_config_tracker->write_drone_radii(v);
        planner.plan_paths();
        // planner.write_states_and_goals(drone_states, goals);
        std::vector<bool> paths_found;
        std::vector<std::vector<Eigen::Vector2d>> paths;

        std::tie(paths_found, paths) = planner.get_paths();
        ws.draw_paths(paths, paths_found);

        if (i==5)
            swarm_config_tracker->write_drone_active_vector(std::vector<bool>({true, true, true, true, false}));

        // std::this_thread::sleep_for(std::chrono::milliseconds(75));
    }

    return 0;
}
