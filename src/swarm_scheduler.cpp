
#include "swarm_scheduler.hpp"


namespace swarm_scheduler{
    void SwarmScheduler::initization(std::vector<std::vector<int>> mission_){
        this->createstatus_list;
        this->setmissions_len(mission_.size());
        this->setmission_idx(mission_.size());
        this->setdrones_len(mission_[0].size());
        this->setdrones_mission(mission_);
        this->createstatus_list(void);
        this->setdrones(void)
        this->missions(void);
    }

    int SwarmScheduler::get_mission_len(){
        return mission_len;
    }

    std::vector<int> SwarmScheduler::getmission_idx(void){
        return mission_idx;
    }

    int SwarmScheduler::getdrone_len(void){
        return drones_len;
    }

    std::vector<std::vector<int>> SwarmScheduler::getdrone_mission(void){
        return drone_mission;
    }

    std:: vector<int> SwarmScheduler::getstatus (void){
        return status;
    }

    std::map<int, std::vector<int>> SwarmScheduler::getdrone_planner(void){
        return drone_planner;
    }
    
    std::map<int, std::vector<int>> SwarmScheduler::getmission_logger(void){
        return mission_logger;
    }

    std::vector<int> SwarmScheduler::getdrones(void){
        return drones_list;
    }
    //set functions

    void SwarmScheduler::setmissions_len(int x){
        mission_len = x;
    }
    void SwarmScheduler::setmission_idx(int x){
        std::vector<int> payload_idx; 
        for(int i=0;i<x;i++){
            payload_idx.push_back(i);
        }
        mission_idx = payload_idx;
    }
    
    void SwarmScheduler::setdrones_len(int  x){
        drones_len = x;
    }


    void SwarmScheduler::setdrones_mission(std::vector<std::vector<int>> x){
        if (x.size() == mission_len && x[0].size()==drones_len){
            drone_mission = x;
        }
        else{
            printf("th matrix size is not same as drone lenght and payload len");
        }
    }
     
    void SwarmScheduler::setdrones(void){
        for(int i =0;i<drones_len){
            drones_list.push_back(i);
        }
    }


    void SwarmScheduler::createstatus_list(){
        status.clear();
        for(int i=0 ; i<mission_len; i++){
            status.push_back(0);
            //status = 0 not started
            //status = 1 pickup done  
            //status = 2 drop off done 
        }
    }

    void SwarmScheduler::radius_create(void){
        for(int i=0;i<drones_len;i++){
            radii_.push_back(0.15);
        }
    }
    
    

    //other methods

    void SwarmScheduler::missions(void){
        int drone;
        std::vector<int> data;
        int payload; 
        int x;
        std::vector<int> temp;
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

    void SwarmScheduler::mission_check(void){
        //int len_ = mission_len; //payload lenght
        int drones_len_ = drones_len; // drones lenght
        int drone_;
        std::vector<int> temp;
        int drone_current_mission;
        int check = 1;
        int reach;
        float final_x,final_y,current_x,current_y;
        float distance;
        int first_check;
        int index_;
        int counter;
        Eigen::Vector2d pay_;
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
                    goals[i](0) = final_x;
                    goals[i](1) = final_y;

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
                        
                        first_check = 0;
                        counter = 0;
                        for(int j = 0;j<drones_len_;j++){
                            if(drone_mission[drone_current_mission][j]==1){
                                if(first_check==0){
                                    index_ = j; 
                                }
                                else{
                                    //change drones active list
                                    drones_active[j]=false;
                                }
                                counter = counter+1;
                            }
                        }
                        //change raduis 
                        radii[index_] = 0.15 + 0.5 *(counter-1); 
                        
                        //update payload_idx with drone_active 
                        //add dict
                        pay_(1)=index_;
                        pay_(0)=drone_current_mission;
                        payload_dict.push_back(pay_)
                    }
                    
                    
                }
                else if (status[drone_current_mission]==1){
                    check = 1;
                    final_x = payload_points[drone_current_mission][3];
                    final_y = payload_points[drone_current_mission][4];
                    
                    //goto drop off
                    goals[i](0) = final_x;
                    goals[i](1) = final_y;

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
                        first_check = 0;
                        for(int j = 0;j<drones_len_;j++){
                            if(drone_mission[drone_current_mission][j]==1){
                                if(first_check==0){
                                    index_ = j; 
                                }
                                else{
                                    //change drones active list
                                    drones_active[j]=true;
                                }
                                //pop mission from the drones in mission
                                drone = drones_list[j];
                                temp = drone_planner[drone];
                                temp.erase(temp.begin());
                                drone_planner[drone] = temp;
                            }
                        }
                        //change raduis 
                        radii_[index_] = 0.15;
                    }   
                }
            }
        }
        swarm_config_tracker->write_drone_goals(goals_);
        swarm_config_tracker->write_drone_active_vector(drones_active);
        swarm_config_tracker->write_drone_radii(radii_);
        


    }
}


