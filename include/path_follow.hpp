#ifndef PATH_FOLLOW_H_
#define PATH_FOLLOW_H_

#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"



std::vector<Eigen::Vector2d> get_drone_velocity_setpoint();
std::vector<Eigen::Vector2d> get_drone_setpoint_error();

std::vector<Eigen::Vector2d> drone_velocity_setpoint;
std::vector<Eigen::Vector2d> drone_setpoint_error;
int kp = 1;






std::vector<Eigen::Vector2d> get_drone_velocity_setpoint(std::vector<Eigen::Vector4d> current_drone_state, std::vector<std::vector<Eigen::Vector2d>> path){
    for (int i =0; i<= path.size(); i++){
        
        

        double current_drone_state_x = current_drone_state[i][0];
        double current_drone_state_y = current_drone_state[i][1];
        double path_x = path[i][0][0];
        double path_y = path[i][0][1];
        double drone_setpoint_error_x = path_x - current_drone_state_x;
        double drone_setpoint_error_y = path_y - current_drone_state_y;
        drone_setpoint_error[i](0) = drone_setpoint_error_x;
        drone_setpoint_error[i](1) = drone_setpoint_error_y;
        drone_velocity_setpoint[i] = kp*drone_setpoint_error[i] ;


            
        
    }
}



#endif