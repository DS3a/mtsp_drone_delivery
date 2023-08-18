
#include<stdio.h>
#include<vector>
#include<cmath>
#include <map>
#include <string.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "include/swarm_planner_deps/swarm_config_tracker.hpp"
#include "include/workspace.hpp"

class mission{
    private:
    int mission_len;
    std::vector<int> mission_idx;
    int drones_len;
    std::vector<std::vector<int>> drone_mission;
    std::vector <int> status;
    std::map<std::string, std::vector<int>> drone_planner;
    std::map<std::string, std::vector<int>> mission_logger;
    std::vector<std::string> drones_list;
    std::vector<Eigen::Vector4d> drone_states;
    std::vector<Eigen::Vector4d> payload_points;

    public:

    int get_mission_len(){
        return mission_len;
    }

    std::vector<int> getmission_idx(void){
        return mission_idx;
    }

    int getdrone_len(void){
        return drones_len;
    }

    std::vector<std::vector<int>> getdrone_mission(void){
        return drone_mission;
    }

    std:: vector<int> getstatus (void){
        return status;
    }

    std::map<std::string, std::vector<int>> getdrone_planner(void){
        return drone_planner;
    }
    
    std::map<std::string, std::vector<int>> getmission_logger(void){
        return mission_logger;
    }

    std::vector<std::string> getdrones(void){
        return drones_list;
    }
    //set functions

    void setmissions_len(int x){
        mission_len = x;
    }
    void setmission_idx(const std::vector<int>& payload_idx_){
        if(payload_idx_.size()==mission_len){
            mission_idx = payload_idx_;
        }
        else{
            printf("the lenght of the mission is not same as lenghtr of mission_idx");
        }
    }
    void setdrones_len(int  x){
        drones_len = x;
    }

    void setdrones_mission(const std::vector<std::vector<int>> x){
        if (x.size() == mission_len && x[0].size()==drones_len){
            drone_mission = x;
        }
        else{
            printf("th matrix size is not same as drone lenght and payload len");
        }
    }
     
    void setdrones(const std::vector<std::string> drone_){
        if(drone_.size() == drones_len ){
            drones_list = drone_;
        }
        else{
            printf("the drone list lenght is not same as drone lenght");
        }
    }


    void createstatus_list(){
        status.clear();
        for(int i=0 ; i<mission_len; i++){
            status.push_back(0);
            //status = 0 not started
            //status = 1 pickup done  
            //status = 2 drop off done 
        }
    }
    
    

    //other methods

    void missions(void){
        std::string drone;
        std::vector<int> data;
        int payload; 
        int x;
        std::vector<int> temp;
        //std::vector<std::string> current_mission_drones;
        data.push_back(-1);
        for(int i = 0; i<drones_len;i++){
            drone = drones_list[i];
            drone_planner[drone] = data;
        }
        for(int i =0; i<mission_len;i++){
            payload = mission_idx[i];
            drone_mission[payload] = data;
        }  
        for(int i=0; i<mission_len;i++){
            //current_mission_drones.clear();
            for(int j=0; j<drones_len;j++){
                if(drone_mission[i][j]==1){
                    x = mission_idx[i]; //
                    drone = drones_list[j];
                    temp = drone_planner[drone];
                    if (temp[0] == -1){
                        temp.clear();
                        temp.push_back(x);
                    }
                    else{
                        temp.push_back(x);
                    }
                    drone_planner[drone] = temp;
                }
            }
        } 
    }

    void mission_check(void){
        int len_ = mission_len; //payload lenght
        int drones_len_ = drones_len; // drones lenght
        std::string drone_;
        std::vector<int> temp;
        int drone_current_mission;
        int check = 1;
        int reach;
        float final_x,final_y,current_x,current_y;
        float distance;
        for(int i=0 ;i<drones_len_;i++){
            drone_ = drones_list[i];
            temp = drone_planner[drone_];
            if(temp.size()==0){
                continue;
            }
            else{
                drone_states = swarm_config_tracker->read_drone_states();
                payload_points = workspace->read_payloads()
                drone_current_mission = temp[0];
                if (status[drone_current_mission] == 0){
                    check = 1;
                    final_x = payload_points[drone_current_mission][0];
                    final_y = payload_points[drone_current_mission][1];
                    
                    //goto pickup point

                    for(int j=0; j<drones_len_;j++){
                        if (drone_mission[drone_current_mission][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            distance = math.sqrt((current_x-final_x)**2+(current_y-final_y)**2);
                            if (distance<=0.01){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check *reach;
                        }
                    }
                    if(check == 1){
                        status[drone_current_mission]=1 
                        //change raduis 
                    }
                    
                    
                }
                else if (status[drone_current_mission]==1){
                    check = 1;
                    final_x = payload_points[drone_current_mission][3];
                    final_y = payload_points[drone_current_mission][4];
                    //goto drop off

                    or(int j=0; j<drones_len_;j++){
                        if (drone_mission[drone_current_mission][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            distance = math.sqrt((current_x-final_x)**2+(current_y-final_y)**2);
                            if (distance<=0.01){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check *reach;
                        }
                    }
                    if(check == 1){
                        status[drone_current_mission]=2 
                        //change raduis 
                        //pop mission from the drones in mission
                    }
                    
                }

                
            }
        }
    }

    void mission_start(){
        //start
    }

};
