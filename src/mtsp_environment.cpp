#include "payload.hpp"
#include "swarm_planner.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include "workspace.hpp"
#include "path_follow.hpp"
#include "base.hpp"
#include <chrono>
#include <thread>
#include<string>
#include<vector>
#include "swarm_scheduler.hpp"
#include "read_csv.hpp"

int main() {
    mtsp_drones_gym::Workspace ws(true);
    std::vector<std::vector<double> > drone_params = read_csv::read_csv_drones("../csv/10drones.csv");
    std::vector<std::vector<double> > payload_params = read_csv::read_csv_payloads("../csv/10payloads.csv");

    for(int k = 0;k<drone_params[0].size();k++){
        ws.add_drone(drone_params[0][k],drone_params[1][k], 0.1, 1);
    }

    for(int w = 0;w<payload_params[0].size();w++){
        ws.add_payload(payload_params[0][w], payload_params[1][w], payload_params[2][w], payload_params[3][w], payload_params[4][w]);
    }
    ws.set_step_time(0.015);
    std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(-1, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0),Eigen::Vector2d(-1, 0), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0)};
 
        
    // std::vector<Eigen::Vector2d> goals = std::vector<Eigen::Vector2d> {Eigen::Vector2d(1, 0)};
    

    //mtsp_drones_gym::Move dronea, droneb, dronec, droned, dronee;
    //dronea = (mtsp_drones_gym::Move) {.x = 0.0, .y = 0.5};
    //droneb = (mtsp_drones_gym::Move) {.x = 2, .y = -4};
    //dronec = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    //droned = (mtsp_drones_gym::Move) {.x = 1, .y = 0.5};
    //dronee = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    //ws.set_actions(std::vector<mtsp_drones_gym::Move>{dronea, droneb, dronec, droned, dronee});

    // ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea});

    std::vector<Eigen::Vector2d> workspace_dims = std::vector<Eigen::Vector2d>();
    workspace_dims.push_back(Eigen::Vector2d(5, -5));
    workspace_dims.push_back(Eigen::Vector2d(5, -5));

    std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker = std::make_shared<swarm_planner::SwarmConfigTracker>();
    swarm_config_tracker->set_num_drones(drone_params[0].size());

    std::vector<Eigen::Vector4d> initial_states_;
    initial_states_.resize(drone_params[0].size());
    std::vector<bool> drones_active;
    drones_active.resize(drone_params[0].size());
    std::vector<double> radii_;
    radii_.resize(drone_params[0].size());
    for(int i =0; i<drone_params[0].size();i++){
        initial_states_[i] = Eigen::Vector4d(drone_params[0][i],drone_params[1][i],0.1,0);
        drones_active[i] = true;
        radii_[i] = 0.1;

    }
    //swarm_config_tracker->write_swarm_config(initial_states_,goals);
    //swarm_config_tracker->write_drone_active_vector(drones_active);
    //swarm_config_tracker->write_drone_radii(radii_);
    
    swarm_config_tracker->write_swarm_config(std::vector<Eigen::Vector4d> {
        Eigen::Vector4d(0, 1.50, 0.1, 0),
        Eigen::Vector4d(0, 1, 0.1, 1),
        Eigen::Vector4d(0, -1, 0.1, 0),
        Eigen::Vector4d(1, -1, 0.1, 0),
        Eigen::Vector4d(-1.5, -1, 0.1, 0),
        Eigen::Vector4d(0, 1.50, 0.1, 0),
        Eigen::Vector4d(0, 1, 0.1, 1),
        Eigen::Vector4d(0, -1, 0.1, 0),
        Eigen::Vector4d(1, -1, 0.1, 0),
        Eigen::Vector4d(-1.5, -1, 0.1, 0)
    }, goals);
    //swarm_config_tracker->write_drone_active_vector(std::vector<bool>({true, true, true, true, true,true, true, true, true, true}));
    swarm_config_tracker->write_drone_active_vector(drones_active);
    swarm_config_tracker->write_drone_radii(radii_);
    //swarm_config_tracker->write_drone_radii(std::vector<double>({0.10, 0.10, 0.10, 0.10, 0.10,0.10, 0.10, 0.10, 0.10, 0.10}));
//
    swarm_planner::SwarmPlannerSE2 planner(workspace_dims, swarm_config_tracker);

    std::vector<Eigen::Vector4d> payload_states;
    std::vector<mtsp_drones_gym::Move> drone_list; 

    ws.set_swarm_config_tracker(swarm_config_tracker);

    std::vector<std::vector<int>> mission_drones_list = 
       {{0, 0, 1, 1, 0, 0, 0, 0, 0, 0}, 
       {0, 1, 0, 0, 0, 0, 0, 0, 0, 1}, 
       {0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 0, 1, 0}, 
        {0, 0, 0, 1, 0, 0, 1, 0, 0, 0}, 
        {1, 0, 1, 0, 0, 0, 1, 0, 0, 0}, 
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0}, 
        {0, 0, 0, 1, 0, 0, 1, 0, 0, 0}, 
        {1, 0, 1, 0, 0, 0, 0, 0, 1, 0}, 
        {1, 0, 1, 0, 0, 0, 0, 0, 1, 0}
        };
    std::vector <int> mission_idx = {7, 8, 6, 5, 3, 1, 0, 4, 2, 9};
    swarm_scheduler::SwarmScheduler sc;
    sc.set_payload_tracker(ws.get_payloads()); // sharing payload pointer with scheduler;
    sc.set_swarm_config_tracker(swarm_config_tracker);
    sc.setmission_idx(mission_idx);
    sc.set_step_time(0.015);
    sc.intilization(mission_drones_list);



    // sc.getpayload_data(ws.read_payloads());
    // sc.set_swarm_config_tracker(swarm_config_tracker);

    for (int i=0; i<10000; i++) {
        sc.print_mission();
        //sc.print_mision_idx();
        //sc.print_payloads();
        sc.print_total_distance();
        sc.print_wait_time();
        sc.mission_check();
       
        auto output = ws.step();
        std::vector<Eigen::Vector4d> drone_states = std::get<1>(output);
        swarm_config_tracker->write_drone_states(drone_states);
        static std::vector<bool> paths_found;
        static std::vector<std::vector<Eigen::Vector2d>> paths;

        if (i % 20 == 0 || i == 0) {
            planner.plan_paths();
            std::tie(paths_found, paths) = planner.get_paths();
        }

        ws.draw_paths(paths, paths_found);


        drone_states = swarm_config_tracker->read_drone_states();
        std::vector<Eigen::Vector2d> drone_setpoints = path_follow::get_drone_velocity_setpoint(drone_states, paths, paths_found);
        std::cout<<"got drone_ setpoints\n";       


        drone_list.clear();
         for (int j=0; j < sc.getdrone_len(); j++) {
             //std::cout << "for path : " << paths[j][1][0] <<  paths[j][1][1] << std::endl;
          
             std::cout << "current x: " << drone_states[j][2] << std::endl;
             std::cout << "current y: " << drone_states[j][3] << std::endl;
             std::cout << "drone setpoints are : " << drone_setpoints[j] << std::endl;
            

             drone_list.push_back((mtsp_drones_gym::Move) {.x = drone_setpoints[j][0],.y = drone_setpoints[j][1]});
         }
        
        std::cout << "gave setpoint " << i << std::endl;
//
        // if (i == 50) {
        //     std::cout << "changing radius of drones\n";
        //     swarm_config_tracker->write_drone_active_vector(std::vector<bool>{false, true, true, true, true});
        //     std::vector<Eigen::Vector4d> read_drone_states = swarm_config_tracker->read_drone_states();
        //     read_drone_states[0] = Eigen::Vector4d(-3, 1, 0, 0);
        //     swarm_config_tracker->write_drone_states(read_drone_states);
        //     // swarm_config_tracker->write_drone_radii(std::vector<double>({0.1, 0.2, 0.1, 0.1, 0.1}));
        // }
        // if (i == 200) {
        //     std::cout << "changing radius of drones\n";
        //     swarm_config_tracker->write_drone_active_vector(std::vector<bool>{true, true, true, true, true});
        //     // swarm_config_tracker->deactivate_drone(0, Eigen::Vector2d(-3, 1));
        //     std::vector<Eigen::Vector4d> read_drone_states = swarm_config_tracker->read_drone_states();
        //     read_drone_states[0] = Eigen::Vector4d(-3, 1, 0, 0);
        //     swarm_config_tracker->write_drone_states(read_drone_states);
        //     // swarm_config_tracker->write_drone_radii(std::vector<double>({0.2, 0.1, 0.1, 0.1, 0.1}));
        // }

        // if (i == 150) {
        //     std::cout << "changing radius of drones\n";
        //     swarm_config_tracker->write_drone_active_vector(std::vector<bool>{true, true, true, true, true});
        //     swarm_config_tracker->write_drone_radii(std::vector<double>({0.1, 0.1, 0.1, 0.1, 0.1}));
        // }


        ws.set_actions(drone_list);
        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

     }


        
   return 0;
}
