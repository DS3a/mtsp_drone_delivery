#ifndef PATH_FOLLOW_H_
#define PATH_FOLLOW_H_

#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"

#define kp 1

namespace path_follow {

std::vector<Eigen::Vector2d> get_drone_velocity_setpoint(std::vector<Eigen::Vector4d> current_drone_state, std::vector<std::vector<Eigen::Vector2d>> path,std::vector<bool> found);








std::vector<Eigen::Vector2d> get_drone_velocity_setpoint(std::vector<Eigen::Vector4d> current_drone_state, std::vector<std::vector<Eigen::Vector2d>> path,std::vector<bool> found){
    std::cout<<"in get_drone_velocuty\n";
    static std::vector<Eigen::Vector2d> drone_velocity_setpoint(path.size());
    std::cout<<"got drone vel\n";
    static std::vector<Eigen::Vector2d> drone_setpoint_error(path.size());
    std::cout<<"got setpoint error\n";
    
    for (int i =0; i< path.size(); i++){
        std::cout<<"path outside: "<<path[i][1][0] <<std::endl;
        std::cout << "in for loop; iteration " << i <<std::endl;
        if (found[i]){
            double current_drone_state_x = current_drone_state[i][0];
            double current_drone_state_y = current_drone_state[i][1];
            std::cout<<"inside got states\n";
            double path_x;
            double path_y;
            if (path[i].size() > 0){
                std::cout<<"okay size"<<i<<std::endl;
                path_x = path[i][1][0];
                path_y = path[i][1][1];

            }
            else {
                std::cout<<"not okay size\n";
                path_x = path[i][0][0];
                path_y = path[i][0][1];
            }
            std::cout<<"inside path is:"<<path_x<<","<<path_y<<std::endl;
            double drone_setpoint_error_x = path_x - current_drone_state_x;
            double drone_setpoint_error_y = path_y - current_drone_state_y;
            std::cout<<"setpoint error:"<<drone_setpoint_error_x<<","<<drone_setpoint_error_y<<std::endl;
            drone_setpoint_error[i][0] = drone_setpoint_error_x;
            drone_setpoint_error[i][1] = drone_setpoint_error_y;
            std::cout<<"added errors insidn\n";
            drone_velocity_setpoint[i] = kp*drone_setpoint_error[i] ;
            std::cout<<"got setpoints:"<<drone_velocity_setpoint[i]<<std::endl;

        }
        else{
            continue;
        }    
    }

    


    

    return drone_velocity_setpoint;
}

}

#endif