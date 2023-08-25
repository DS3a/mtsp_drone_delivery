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
            std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker_;
            std::vector<Eigen::Vector4d> payload_points={{0,0,0,0}};
            std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads_;
            
            std::vector<Eigen::Vector4d> initialstate={{0,0,0,0}};

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
            void set_payload_tracker(std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads);
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
            void getpayload_data(int lenght);
            std::vector<Eigen::Vector4d> read_payload();
            void print_mission();
            void print_payloads();
            friend class SwarmConfigTracker;




    };
 // namespace swarm_scheduler

    void SwarmScheduler::set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker) {
        swarm_config_tracker_ = swarm_config_tracker;
    }

    void SwarmScheduler::set_payload_tracker(std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads){
        this->payloads_ = payloads;
    }

    void SwarmScheduler::intilization(std::vector<std::vector<int>> mission_){
        this->set_swarm_config_tracker(this->swarm_config_tracker_);
        this->set_payload_tracker(this->payloads_);
        this->setmissions_len(mission_.size());
        this->mission_idx.resize(mission_.size());
        this->setmission_idx(mission_.size());
        this->setdrones_len(mission_[0].size());
        this->drone_mission.resize(this->mission_len,std::vector<int>(this->drones_len));
        this->setdrones_mission(mission_);
        this->status.resize(this->mission_len);
        this->createstatus_list();
        this->setdrones();
        this->missions();
        this->payload_points.resize(this->mission_len);
        this->payloads_.resize(this->mission_len);
        this->initialstate.resize(this->drones_len);
        this->initialstate = this->swarm_config_tracker_->read_drone_states();
        printf("completed initilization of mission");
    }

    int SwarmScheduler::get_mission_len(){
        return this->mission_len;
    }

    std::vector<int> SwarmScheduler::getmission_idx(void){
        return this->mission_idx;
    }

    int SwarmScheduler::getdrone_len(void){
        return this->drones_len;
    }

    std::vector<std::vector<int>> SwarmScheduler::getdrone_mission(void){
        return this->drone_mission;
    }

    std:: vector<int> SwarmScheduler::getstatus (void){
        return this->status;
    }

    std::map<int, std::vector<int>> SwarmScheduler::getdrone_planner(void){
        return this->drone_planner;
    }
    
    std::map<int, std::vector<int>> SwarmScheduler::getmission_logger(void){
        return this->mission_logger;
    }

    std::vector<int> SwarmScheduler::getdrones(void){
        return this->drones_list;
    }


    std::vector<Eigen::Vector4d> SwarmScheduler::read_payload(){
        return this->payload_points;
    }
    //set functions

    
    void SwarmScheduler::setmissions_len(int x){
        this->mission_len = x;
    }
    void SwarmScheduler::setmission_idx(int x){
        std::vector<int> payload_idx; 
        for(int i=0;i<x;i++){
            payload_idx.push_back(i);
        }
        this->mission_idx = payload_idx;
    }
    
    void SwarmScheduler::setdrones_len(int  x){
        this->drones_len = x;
    }


    void SwarmScheduler::setdrones_mission(std::vector<std::vector<int>> x){
        if (x.size() == mission_len && x[0].size()==drones_len){
            this->drone_mission = x;
        }
        else{
            printf("th matrix size is not same as drone lenght and payload len");
        }
    }
     
    void SwarmScheduler::setdrones(void){
        this->drones_list.resize(this->drones_len);
        for(int i =0;i<this->drones_len;i++){
            this->drones_list.push_back(i);
        }
    }


    void SwarmScheduler::createstatus_list(){
        this->status.clear();
        for(int i=0 ; i<this->mission_len; i++){
            this->status.push_back(0);
            //status = 0 not started
            //status = 1 pickup done  
            //status = 2 drop off done 
        }
    }
    
    void SwarmScheduler::getpayload_data(int lenght){
        for(int i =0;i<lenght;i++){
            this->payload_points[i] = this->payloads_[i]->get_start_and_dest();
            std::cout<<" added payload : "<<i<<std::endl;
        }  
    }

    
    //other methods
    void SwarmScheduler::print_mission(){
        std::vector<int> x;
        for(int j =0; j<this ->drones_len;j++){
            x = (this->drone_planner[j]);
            for(int i=0;i<x.size();i++){
                std::cout<<x.at(i)<< " ";
            }
            std::cout<<std::endl;
        }    
    }

    void SwarmScheduler::print_payloads(){
        Eigen::Vector4d data;
        for(int i =0; i<this->mission_len;i++){
            data = this->payload_points[i];
            std::cout<<data(0)<<" "<<data(1)<<" "<<data(2)<<" "<<data(3)<<std::endl;
            std::cout<<"printted payload" << i<< std::endl;
        }
    }



    void SwarmScheduler::missions(void){
        std::vector<int> temp;
        temp.resize(mission_len);
        temp.clear();
        temp.push_back(-1);
        for(int i = 0; i<=this->drones_len;i++){
            this->drone_planner[this->drones_list[i]] = temp;
            //initiliziation of drone_planner
        }
        for(int i =0; i<this->mission_len;i++){
            this->mission_logger[this->mission_idx[i]] = temp;
            //initilization of mission planner
        }  
        for(int i=0; i<this->mission_len;i++){
            //current_mission_drones.clear();
            for(int j=0; j<this->drones_len;j++){
                
                if(drone_mission[i][j]==1){
                    std::cout<<"drone :" << j <<" doing mission"<< i<<std::endl; 
                    temp = this->drone_planner[j];
                    if (temp[0] == -1){
                        temp.clear();
                        temp.push_back(i);
                    }
                    else{
                        temp.push_back(i);
                    }
                    this->drone_planner[j] = temp;
                }
            }
        } 
        std::cout<<"done all mission\n";
    }

    void SwarmScheduler::mission_check(void){

        std::vector<int> temp;
        int drone_current_mission;
        int check = 1;
        int reach = 0;
        double final_x,final_y,current_x,current_y;
        float distance;
        int first_check;
        int index_;
        float counter;
        std::vector<double> radii_;
        std::vector<bool> drones_active;
        
        radii_ = this->swarm_config_tracker_->read_drone_radii();
        drones_active = this->swarm_config_tracker_->read_drone_active();
        std::vector<Eigen::Vector4d> drone_states;
        Eigen::Vector2d pay_;
        std::vector<Eigen::Vector2d> goals_(mission_len);
        std::cout<<"intilized variables"<<std::endl;
        drone_states = this->swarm_config_tracker_->read_drone_states();
        
        for(int i=0 ;i<this->drones_len;i++){
            temp = this->drone_planner[i];
            if(temp.size()==0){
                goals_[i](0) = this->initialstate[i][0];
                goals_[i](1) = this->initialstate[i][1];
                continue;
            }
            else if(temp[0]==-1){
                std::cout<<" no mission intilized"<<std::endl;
                goals_[i](0) = drone_states[i][0];
                goals_[i](1) = drone_states[i][1];
                std::cout<<"goal given"<<std::endl;
                //drones_active[i]=false;

            }
            else{
                std::cout<<"in else"<<std::endl;
                // payload_points
                drone_current_mission = temp[0];
                std::cout<<"current mission"<<" "<<drone_current_mission<<std::endl;
                std::cout<<"status of current mission"<<" "<<status[drone_current_mission]<<std::endl;
                if (status[drone_current_mission] == 0) {
                    check = 1;
                    std::cout<<"in status 0 condition"<<std::endl;
                    final_x = this->payload_points[drone_current_mission][0];
                    final_y = this->payload_points[drone_current_mission][1];
                    std::cout<<"got pickup x and y"<<final_x<<final_y<<std::endl;
                    //goto pickup point
                    goals_[i](0) = final_x;
                    goals_[i](1) = final_y;
                    drones_active[i] = true;
                    std::cout<<"assigned pick up goals"<<std::endl;
                    for(int j=0; j<this->drones_len;j++) {
                        if (drone_mission[drone_current_mission][j]==1) {
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            std::cout<<"got current x and y"<<current_x<<current_y<<std::endl;
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
                            if (distance<=0.55){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check *reach;
                        }
                    }
                    std::cout<<"drone "<< i <<" done checking in status 0 :";
                    std::cout<<check<<std::endl;
                    if(check == 1){
                        this->status[drone_current_mission] = 1;
                        counter = 0.1;
                        first_check = 0;
                        for(int j = this->drones_len -1;j>=0;j--){
                            if(this->drone_mission[drone_current_mission][j]==1){
                                //change drones active li
                                
                                if(first_check ==0){
                                    index_= j;
                                    first_check = 1;
                                }
                                else{
                                    drones_active[j]=false;
                                    // goals_[i](0) = this->payload_points[drone_current_mission][2];
                                    // goals_[i](1) = this->payload_points[drone_current_mission][3];
                                    std::cout << "deactivating drones, setting their destination to ";
                                    Eigen::Vector2d payload_dest(this->payload_points[drone_current_mission][2], this->payload_points[drone_current_mission][3]);
                                    std::cout << payload_dest << std::endl;
                                    this->swarm_config_tracker_->deactivate_drone(j,payload_dest);
                                }

                                counter += 0.05;
                            }
                        std::cout<<"radii:"<<j<<" is "<<radii_[j]<<" "<<drones_active[j]<<std::endl;
                        }

                        radii_[index_] = counter;
                        drones_active[index_] = true;
                    }
                    
                    
                }
                else if (status[drone_current_mission]==1){
                    check = 1;
                    std::cout<<"in status 1\n";
                    std::cout <<"current mission "<<drone_current_mission<<std::endl;
                    //std::cout << "x :" <<this->payload_points[drone_current_mission][2]<<std::endl;
                    //std::cout << "y :" <<this->payload_points[drone_current_mission][3]<<std::endl;
                    final_x = this->payload_points[drone_current_mission][2];
                    final_y = this->payload_points[drone_current_mission][3];
                    //goto drop off
                    goals_[i](0) = this->payload_points[drone_current_mission][2];
                    goals_[i](1) = this->payload_points[drone_current_mission][3];
                    std::cout<<"given new goals\n";

                    for(int j=0; j<this->drones_len;j++){
                        if (drone_mission[drone_current_mission][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            pay_(0) = current_x;
                            pay_(1) = current_y;
                            this->payloads_[drone_current_mission]->write_position(pay_);
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
                            std::cout<<"distance "<<distance<<std::endl;
                            if (distance<=0.01){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check * reach;
                        }
                    }
                    if(check == 1){
                        status[drone_current_mission]=2; 
                        for(int j = 0;j<this->drones_len;j++){
                            if(drone_mission[drone_current_mission][j]==1){ 
                                radii_[j] = 0.1;
                                goals_[i](0) = this->payload_points[drone_current_mission][2];
                                goals_[i](1) = this->payload_points[drone_current_mission][3];
                                std::cout << "reactivating drones now\n";
                                std::cout << "the new drone position is " << goals_[i] << "\n";
                                // this->swarm_config_tracker_->deactivate_drone(j, goals_[i]);
                                drones_active[j]=true;
                                //pop mission from the drones in mission
                                temp = drone_planner[j];
                                temp.erase(temp.begin());
                                drone_planner[j] = temp;
                            }
                        }
                        //change raduis 
                        
                    }   
                }
            }
        }
        std::cout<<"completed, yet to write"<<std::endl;
        swarm_config_tracker_->write_drone_goals(goals_);
        std::cout<<"wrote drone goals"<<std::endl;
        swarm_config_tracker_->write_drone_active_vector(drones_active);
        std::cout<<"written drone goals\n";
        swarm_config_tracker_->write_drone_radii(radii_);
        std::cout<<"written drone radii\n";



    }
}

#endif  //SWARM_SCHEDULER_H_

