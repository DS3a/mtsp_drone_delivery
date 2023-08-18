#include "payload.hpp"
#include "swarm_planner.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include "workspace.hpp"
#include "path_follow.hpp"
#include "base.hpp"
#include <chrono>
#include <thread>
#include<string>
int main() {
    mtsp_drones_gym::Workspace ws(true);
    ws.add_drone(0, 0, 0.2, 1);
    ws.add_drone(0, 1, 0.1, 1);
    ws.add_drone(0, -1, 0.1, 1);
    ws.add_drone(-1, -1, 0.1, 1);
    ws.add_drone(-1.5, -1, 0.1, 1);
    ws.set_step_time(0.015);
    

    std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(0, 0), Eigen::Vector2d(0, -0), Eigen::Vector2d(0, -0), Eigen::Vector2d(0, 0), Eigen::Vector2d(-0, 0)};
    // std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 0)};



    // std::vector<std::string> drone_list;
    // for(int i = 0;i <= std::vector::size(paths);i++){

    //     std::string str = "drone";
    //     std::string str2 = i;
    //     str.append(str2);
    //     drone_list[i] = str;
    //     mtsp_drones_gym::Move drone_list[i]; 
    // }
   
        
    // mtsp_drones_gym::Move dronea, droneb, dronec, droned, dronee;
    //dronea = (mtsp_drones_gym::Move) {.x = 0.5, .y = 0.5};
    //droneb = (mtsp_drones_gym::Move) {.x = 2, .y = -4};
    //dronec = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    //droned = (mtsp_drones_gym::Move) {.x = 1, .y = 0.5};
    //dronee = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    //ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea, droneb, dronec, droned, dronee});
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

    swarm_planner::SwarmPlannerSE2 planner(workspace_dims, swarm_config_tracker);

    std::vector<Eigen::Vector4d> payload_states;
    std::vector<mtsp_drones_gym::Move> drone_list; 

    for (int i=0; i<100; i++) {
        auto output = ws.step();
        std::vector<Eigen::Vector4d> drone_states = std::get<1>(output);
        swarm_config_tracker->write_swarm_config(drone_states, goals);
        planner.plan_paths();
        // planner.write_states_and_goals(drone_states, goals);
        std::vector<bool> paths_found;
        std::vector<std::vector<Eigen::Vector2d>> paths;

        std::tie(paths_found, paths) = planner.get_paths();
        drone_states = swarm_config_tracker->read_drone_states();
        
        ws.draw_paths(paths);

        std::vector<Eigen::Vector2d> drone_setpoints = get_drone_velocity_setpoint(drone_states, paths);
              

        for (int i=0; i < drone_setpoints.size(); i++) {
            drone_list.push_back((mtsp_drones_gym::Move) {.x = drone_setpoints[i][0],.y = drone_setpoints[i][1]});
        }

     }

        ws.set_actions(drone_list);


        // std::this_thread::sleep_for(std::chrono::milliseconds(75));
   return 0;
}
