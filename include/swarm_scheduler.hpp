#ifndef SWARM_SCHEDULER_H_
#define SWARM_SCHEDULER_H_


#include<stdio.h>
#include<vector>
#include<cmath>
#include <map>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

namespace swarm_scheduler{
    class SwarmScheduler{
        private:
            int mission_len = 0 ; //payload len
            std::vector<int> mission_idx ={0}; //payload id
            int drones_len = 0; //no of drones
            std::vector<std::vector<int>> drone_mission={{0}}; //list of list of matrix
            std::vector <int> status={0}; 
            std::map<int, std::vector<int>> drone_planner;
            std::map<int, std::vector<int>> mission_logger;
            std::vector<int> drones_list={0};
            std::vector<Eigen::Vector2d> payload_dict={{0,0}}; // payload_id, drone_id
            std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker_;
            std::vector<Eigen::Vector4d> payload_points={{0,0,0,0}};

        public:

            void intilization(std::vector<std::vector<int>> mission_);
            //getter functions
            int get_mission_len();
            std::vector<int> getmission_idx();
            int getdrone_len();
            std::vector<std::vector<int>> getdrone_mission();
            std::vector<int>getstatus(void);
            std::map<int,std::vector<int>> getdrone_planner();
            std::map<int,std::vector<int>> getmission_logger();
            std::vector<int> getdrones();
            void set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker);
            void Workspace();
            //setter functions
            void setmissions_len(int x);
            void setmission_idx(int x);
            void setdrones_len(int x);
            void setdrones_mission(std::vector<std::vector<int>> x);
            void setdrones(void);
            void createstatus_list();

            void missions();
            void mission_check(void);
            void getpayload_data(std::vector<Eigen::Vector4d> payload_points_);
            std::vector<Eigen::Vector4d> read_payload();
            void print_mission();

            void print_payloads();
            friend class SwarmConfigTracker;




    };
 // namespace swarm_scheduler

    void SwarmScheduler::set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker) {
        swarm_config_tracker_ = swarm_config_tracker;
    }

    void SwarmScheduler::intilization(std::vector<std::vector<int>> mission_){
        this->set_swarm_config_tracker(this->swarm_config_tracker_);
        this->setmissions_len(mission_.size());
        this->setmission_idx(mission_.size());
        this->setdrones_len(mission_[0].size());
        this->setdrones_mission(mission_);
        this->createstatus_list();
        this->setdrones();
        this->missions();
        printf("completed initilization of mission");
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

    std::vector<Eigen::Vector4d> SwarmScheduler::read_payload(){
        return payload_points;
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
        for(int i =0;i<drones_len;i++){
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
    
    void SwarmScheduler::getpayload_data(std::vector<Eigen::Vector4d> payload_){
        payload_points = payload_;  
    }

    
    //other methods
    void SwarmScheduler::print_mission(){
        std::vector<int> x;
        for(int j =0; j<drones_len;j++){
            x = (drone_planner[j]);
            for(int i=0;i<x.size();i++){
                std::cout<<x.at(i)<< " ";
            }
            std::cout<<std::endl;
        }    
    }

    void SwarmScheduler::print_payloads(){
        Eigen::Vector4d data;
        for(int i =0; i<payload_points.size();i++){
            data = payload_points[i];
            std::cout<<data(0)<<" "<<data(1)<<" "<<data(2)<<" "<<data(3)<<std::endl;
        }
    }

    

    void SwarmScheduler::missions(void){
        int drone;
        std::vector<int> data;
        int payload; 
        int x;
        std::vector<int> temp;
        data.push_back(-1);
        for(int i = 0; i<=drones_len;i++){
            drone = drones_list[i];
            drone_planner[drone] = data;
            //initiliziation of drone_planner
        }
        for(int i =0; i<mission_len;i++){
            payload = mission_idx[i];
            mission_logger[payload] = data;
            //initilization of mission planner
        }  
        for(int i=0; i<mission_len;i++){
            //current_mission_drones.clear();
            for(int j=0; j<drones_len;j++){
                
                if(drone_mission[i][j]==1){
                    std::cout<<"drone :" << j <<" doing mission"<< i<<std::endl;
                    x = i; //
                    drone = j;
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
        std::vector<int> temp;
        int drone_current_mission;
        int check = 1;
        int reach;
        double final_x,final_y,current_x,current_y;
        float distance;
        int first_check;
        int index_;
        int counter;
        std::vector<Eigen::Vector4d> drone_states;
        Eigen::Vector2d pay_;
        std::vector<Eigen::Vector2d> goals_(mission_len);
        std::vector<double> radii_(drones_len);
        std::vector<bool> drones_active(drones_len);
        std::cout<<"intilized variables"<<std::endl;
        drone_states = this->swarm_config_tracker_->read_drone_states();
        //std::cout<<"got drone_states"<<std::endl;

        for(int i=0; i<drones_len;i++){
            radii_[i] = 0.1;
        }
        for(int i=0 ;i<drones_len_;i++){
            //std::cout<<"inn for loop ";
            /temp = drone_planner[i];
            //std::cout<<temp[0]<<std::endl;
            //std::cout<<temp.size()<<std::endl;
            if(temp.size()==0){
                continue;
            }
            else if(temp[0]==-1){
                std::cout<<" no mission intilized"<<std::endl;
                goals_[i](0) = drone_states[i][0];
                goals_[i](1) = drone_states[i][1];
                std::cout<<"goal given"<<std::endl;
                drones_active[i]=false;

            }
            else{
                std::cout<<"in else"<<std::endl;
                // payload_points
                drone_current_mission = temp[0];
                std::cout<<"current mission"<<" "<<drone_current_mission<<std::endl;
                std::cout<<"status of current mission"<<" "<<status[drone_current_mission]<<std::endl;
                if (status[drone_current_mission] == 0){
                    check = 1;
                    std::cout<<"in status 0 condition"<<std::endl;
                    final_x = payload_points[drone_current_mission][0];
                    final_y = payload_points[drone_current_mission][1];
                    std::cout<<"got pickup x and y"<<final_x<<final_y<<std::endl;
                    //goto pickup point
                    goals_[i](0) = final_x;
                    goals_[i](1) = final_y;
                    drones_active[i] = true;
                    std::cout<<"assigned pick up goals"<<std::endl;
                    for(int j=0; j<drones_len_;j++){
                        if (drone_mission[drone_current_mission][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            std::cout<<"got current x and y"<<current_x<<current_y<<std::endl;
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
                            if (distance<=0.01){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check *reach;
                        }
                    }
                    std::cout<<"drone "<< i <<" done checking in status 0"<<std::endl;
                    std::cout<<check<<std::endl;
                    if(check == 1){
                        status[drone_current_mission]=1;
                        
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
                        radii_[index_] = 0.1 + 0.5 *(counter-1); 
                        
                        //update payload_idx with drone_active 
                        //add dict
                        pay_(1)=index_;
                        pay_(0)=drone_current_mission;
                        payload_dict.push_back(pay_);
                    }
                    
                    
                }
                else if (status[drone_current_mission]==1){
                    check = 1;
                    final_x = payload_points[drone_current_mission][3];
                    final_y = payload_points[drone_current_mission][4];
                    
                    //goto drop off
                    goals_[i](0) = final_x;
                    goals_[i](1) = final_y;

                    for(int j=0; j<drones_len_;j++){
                        if (drone_mission[drone_current_mission][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
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
                        status[drone_current_mission]=2; 
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
                                temp = drone_planner[j];
                                temp.erase(temp.begin());
                                drone_planner[j] = temp;
                            }
                        }
                        //change raduis 
                        radii_[index_] = 0.1;
                    }   
                }
            }
        }
        std::cout<<"completed, yet to write"<<std::endl;
        swarm_config_tracker_->write_drone_goals(goals_);
        std::cout<<"wrote drone goals"<<std::endl;
        swarm_config_tracker_->write_drone_active_vector(drones_active);
        
        swarm_config_tracker_->write_drone_radii(radii_);
        


    }
}

#endif  //SWARM_SCHEDULER_H_

