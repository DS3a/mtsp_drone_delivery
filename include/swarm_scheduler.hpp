#ifndef SWARM_SCHEDULER_H_
#define SWARM_SCHEDULER_H_


#include<stdio.h>
#include<vector>
#include<cmath>
#include <map>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <cstdlib>
#include <numeric>


namespace swarm_scheduler{
    class SwarmScheduler{
        private:
            int mission_len = 0 ; //payload len
            std::vector<int> mission_idx ={0}; //payload id
            int drones_len = 0; //no of drones
            std::vector<std::vector<int>> drone_mission={{0}}; //list of list of matrix
            std::vector <int> status={0}; //status of each mission
            std::map<int, std::vector<int>> drone_planner; // queue of misiion in each drone
            std::vector<int> drones_list={0}; //list of drones
            std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker_; // pointer to swarm_config_tracker
            std::vector<std::vector<double>> payload_location={{0}};
            std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads_; //payload pointer
            std::vector<Eigen::Vector4d> initialstate={{0,0,0,0}}; // initialstate 
            double total_time = 0; // total time of drones
            double step_time = 0.015; // step time of each mission
            std::vector<Eigen::Vector4d> previous_state ={{0,0,0,0}};
            std::vector<double> total_distance={0};
            std::vector<Eigen::Vector2d> goal;
            std::vector<bool> goal_status; 
            std::vector<bool> drones_reached;

        public:

            void intilization(std::vector<std::vector<int>> mission_);
            //getter functions
            int get_mission_len();
            std::vector<int> getmission_idx();
            int getdrone_len();
            std::vector<std::vector<int>> getdrone_mission();
            std::vector<int>getstatus(void);
            std::map<int,std::vector<int>> getdrone_planner();
            std::vector<int> getdrones();
            void set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker);
            void set_payload_tracker(std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads);
            double get_total_time();
            //setter functions
            void setmissions_len(int x);
            void setmission_idx(std::vector<int> x);
            void setdrones_len(int x);
            void setdrones_mission(std::vector<std::vector<int>> x);
            void setdrones(void);
            void createstatus_list();
            void set_step_time(double step);
            void set_total_distance();

            void missions();
            void mission_check(void);
            void print_mission();
            double point(double x,double y);
            Eigen::Vector2d point_genrator(double x,double y);
            void print_mision_idx();
            void print_total_time();
            void set_previous_state(std::vector<Eigen::Vector4d> state);
            void print_total_distance();
            void payload_splitter(int x);
            void print_payload();
            bool update_total_time();
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
        std::cout<<"got swarm config pointer\n";
        this->set_payload_tracker(this->payloads_);
        std::cout<<"got payload pointer\n";
        this->setmissions_len(mission_.size());
        this->mission_idx.resize(mission_.size());
        this->setdrones_len(mission_[0].size());
        this->drone_mission.resize(this->mission_len,std::vector<int>(this->drones_len));
        this->setdrones_mission(mission_);
        this->status.resize(this->mission_len);
        this->createstatus_list();
        this->setdrones();
        this->goal_status.resize(this->drones_len);
        this->drones_reached.resize(this->drones_len);
        this->goal.resize(this->drones_len);
        this->missions();
        this->payloads_.resize(this->mission_len);
        this->initialstate.resize(this->drones_len);
        this->initialstate = this->swarm_config_tracker_->read_drone_states();
        this->payload_location.resize(this->mission_len,std::vector<double>(1));
        this->payload_splitter(this->mission_len);
        this->previous_state.resize(this->drones_len);
        this->set_previous_state(this->initialstate);
        this->total_distance.resize(this->drones_len);
        
        //this->payload_location.resize(this->mission_len,std::vector<double>(this->drones_len));
        
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
    
    std::vector<int> SwarmScheduler::getdrones(void){
        return this->drones_list;
    }

    double SwarmScheduler::get_total_time(){
        return this->total_time;
    }

    //set functions

    
    void SwarmScheduler::setmissions_len(int x){
        this->mission_len = x;
        std::cout<<"done setting mission len\n";
    }
    void SwarmScheduler::setmission_idx(std::vector<int> payload_idx){
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
    

    void SwarmScheduler::set_step_time(double step){
        this->step_time = step;
    }

    void SwarmScheduler::set_previous_state(std::vector<Eigen::Vector4d> state){
        this->previous_state = state;
    }
    
    void SwarmScheduler::set_total_distance(){
        for(int i =0; i<this->drones_len;i++){
            this->total_distance.push_back(0.0);
        }
    }
    //other methods
    void SwarmScheduler::print_total_time(){
        std::cout<<this->total_time<<std::endl;
    }

    void  SwarmScheduler::print_mision_idx(){
        for(int i=0; i<this->mission_len; i++){
            std::cout<<this->mission_idx[i]<<" ";
        }
        std::cout<<" \n";
    }

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

    bool SwarmScheduler::update_total_time(){
        bool temp = false;
        for(int i=0;i<this->drones_len; i++){
            temp = temp | this->drones_reached[i];
        }
        if(temp == true){
            this->total_time+=this->step_time;
        }
        return temp;
    }


    void SwarmScheduler::print_total_distance(){
        float distance =0;
        std::cout<<"total_dist: ";
        for(int i =0;i<this->drones_len;i++){
            distance += this->total_distance[i];
        }
        std::cout<<distance<<std::endl;
    }
    double SwarmScheduler::point(double x,double y){
        double mean = x;
        double variance = y;
        double  u1,u2,z0;
        do{
        u1 = rand() / (double)RAND_MAX;
        u2 = rand() / (double)RAND_MAX;
        } while(u1 <= 1e-7);

        double mag = std::sqrt(-2.0*std::log(u1));
        z0 = mag * std::cos(2 * M_PI * u2);
        return z0*std::sqrt(variance) + mean;
    }

    Eigen::Vector2d SwarmScheduler::point_genrator(double x,double y){
        Eigen::Vector2d random_point;
        random_point(0) = this->point(x,y);
        random_point(1) = this->point(x,y);
        return random_point;
    }

    void SwarmScheduler::print_payload(){
        for(int i =0; i<this->mission_len;i++){
            std::cout<<this->payload_location[i].size()<<"  ";
            for(int j =0; j<this->payload_location[i].size();j++){
                std::cout<<this->payload_location[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }


    
    void SwarmScheduler::payload_splitter(int lenght){
        Eigen::Vector4d payload_points;
        int index,number,counter;
        std::vector<int> current_mission;
        const double PI = 3.14159265358979323846;
        double r = 0.3;
        current_mission.resize(this->drones_len);
        for(int i =0;i<lenght;i++){
            
            payload_points = this->payloads_[i]->get_start_and_dest();
            for(int k=0; k<this->mission_len;k++){
                    if(i==this->mission_idx[k]){
                        index = k;
                    }
                }
            //index = this->mission_idx[i];
            std::cout<<i<<"in index : "<< index<<std::endl;
            current_mission =this->drone_mission[index];
            counter =0;
            for(int j=0; j<this->drones_len;j++){
                counter +=current_mission[j];
            }
            std::cout<<"sum is: "<<counter<<std::endl;
            if(counter == 1){
                this->payload_location[i]={payload_points[0],payload_points[1],payload_points[0],payload_points[1],payload_points[2],payload_points[3]};
                std::cout<<" added goals "<<counter<<std::endl;
            }
            else if(counter == 0){
                this->payload_location[i]={payload_points[0],payload_points[1],payload_points[2],payload_points[3]};
                std::cout<<" added goals "<<counter<<std::endl;
            }
            else{
                this->payload_location[i].resize(0);
                this->payload_location[i].clear();
                //this->payload_location[i].push_back(0);
                double angle = 2 * PI / counter;
                for (int k = 0; k < counter; ++k) {
                    double theta = k * angle;
                    double px = payload_points[0] + r * cos(theta);
                    this->payload_location[i].push_back(px);
                    double py = payload_points[1] + r * sin(theta);
                    this->payload_location[i].push_back(py);
                    std::cout<<" added"<<px<< "and" <<py;
                }
                this->payload_location[i].push_back(payload_points[0]);
                this->payload_location[i].push_back(payload_points[1]);
                this->payload_location[i].push_back(payload_points[2]);
                this->payload_location[i].push_back(payload_points[3]);
                std::cout<<this->payload_location[i].size() <<"added final points\n";
            }
        }
    }


    void SwarmScheduler::missions(void){
        std::vector<int> temp;
        temp.resize(mission_len);
        temp.clear();
        temp.push_back(-1);
        for(int i = 0; i<=this->drones_len;i++){
            this->drone_planner[this->drones_list[i]] = temp;
            this->goal_status[i] = true; 
            this->drones_reached[i] = true;
            //initiliziation of drone_planner
        } 
        for(int i=0; i<this->mission_len;i++){
            int mission_data = this->mission_idx[i];
            for(int j=0; j<this->drones_len;j++){
                if(drone_mission[i][j]==1){
                    temp = this->drone_planner[j];
                    if (temp[0] == -1){
                        temp.clear();
                        temp.push_back(mission_data);
                    }
                    else{
                        temp.push_back(mission_data);
                    }
                    this->drone_planner[j] = temp;
                }
            }
        } 
        std::cout<<"done mission\n";
    }



    void SwarmScheduler::mission_check(void){

        std::vector<int> temp;
        int drone_current_mission_idx,drone_current_mission;
        int check = 1;
        int reach = 0;
        double final_x,final_y,current_x,current_y,previous_x,previous_y;
        float distance;
        int first_check;
        int index_;
        float counter;
        std::vector<double> radii_;
        std::vector<bool> drones_active;
        radii_ = this->swarm_config_tracker_->read_drone_radii();
        drones_active = this->swarm_config_tracker_->read_drone_active();
        std::cout<<"drones_active";
        for(int i=0; i<this->drones_len;i++){
            std::cout<<drones_active[i]<<" ";
        }
        std::cout<<std::endl;
        std::vector<Eigen::Vector4d> drone_states;
        Eigen::Vector2d pay_;
        std::vector<Eigen::Vector2d> goals_(this->drones_len);
        drone_states = this->swarm_config_tracker_->read_drone_states();
        int sign_data =-1;
        for(int i=0 ;i<this->drones_len;i++){
            temp = this->drone_planner[i];
            if(temp.size()==0){
                if (this->goal_status[i] == true){
                    this->goal[i](0) = this->initialstate[i][0];
                    this->goal[i](1) = this->initialstate[i][1];
                    this->goal_status[i] = false; 

                }
                current_x = drone_states[i][0];
                current_y = drone_states[i][1];
                previous_x = this->previous_state[i][0];
                previous_y = this->previous_state[i][1];
                distance = sqrt(pow((current_x-previous_x),2)+pow((current_y-previous_y),2));
                this->total_distance[i]+= distance; 
                double distance_reach = sqrt(pow((current_x-this->initialstate[i][0]),2)+pow((current_y-this->initialstate[i][1]),2));
                if(distance_reach<0.05){
                    this->drones_reached[i]=false;
                }
            }
            else if(temp[0]==-1){
                if(this->goal_status[i]==true){
                    this->goal[i](0) = drone_states[i](0);
                    this->goal[i](1) = drone_states[i][1];
                    this->goal_status[i] = false; 
                    this->drones_reached[i] = false;
                }
            }
            else{
                std::cout<<"in else"<<std::endl;
                drone_current_mission = temp[0];
                for(int k=0; k<this->mission_len;k++){
                    if(drone_current_mission==this->mission_idx[k]){
                        drone_current_mission_idx = k;
                    }
                }

                std::cout<<"current mission"<<" "<<drone_current_mission_idx<<" "<<drone_current_mission<<std::endl;
                std::cout<<"status of current mission"<<" "<<status[drone_current_mission_idx]<<std::endl;
                int counter = 0;
                if (status[drone_current_mission_idx] == 0) {
                    check = 1;
                    //int sum_ = 0;
                    //sum_ = accumulate(drone_mission[drone_current_mission].begin(), drone_mission[drone_current_mission].end(), 0);
                    std::cout<<"in status 0 condition"<<std::endl;
                    final_x = this->payload_location[drone_current_mission][0];
                    final_y = this->payload_location[drone_current_mission][1];
                    std::cout<<"got pickup x and y"<<final_x<<final_y<<std::endl;
                    
                    //goto pickup point
                    if(this->goal_status[i]==true){
                        final_x = this->payload_location[drone_current_mission][0];
                        final_y = this->payload_location[drone_current_mission][1];
                        this->goal[i](0) = final_x ;
                        this->goal[i](1) = final_y ;   
                        this->payload_location[drone_current_mission].erase(this->payload_location[drone_current_mission].begin());
                        this->payload_location[drone_current_mission].erase(this->payload_location[drone_current_mission].begin());
                        //this->payload_location[drone_current_mission].pop_front();
                        //this->payload_location[drone_current_mission].pop_front();
                        this->goal_status[i] = false;
                        std::cout<<"given goal: "<<final_x<<" "<<final_y<<std::endl;

                    }
                    for(int j=0; j<this->drones_len;j++) {
                        std::cout<<"in for loop\n";
                        
                        if (drone_mission[drone_current_mission_idx][j]==1 ) {
                            std::cout<<"in if condition: "<<j<<std::endl;
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            std::cout<<"current"<<current_x<<current_y<<std::endl;
                            if(i==j){
                                double distance_;
                                previous_x = this->previous_state[j][0];
                                previous_y = this->previous_state[j][1];
                                distance_ = sqrt(pow((current_x-previous_x),2)+pow((current_y-previous_y),2));
                                this->total_distance[j]+=distance_; 
                            }
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
                            std::cout<<distance<<" "<<j<<std::endl;
                            if (distance<= 0.5){
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
                        this->payload_location[drone_current_mission].erase(this->payload_location[drone_current_mission].begin());
                        this->payload_location[drone_current_mission].erase(this->payload_location[drone_current_mission].begin());
                        this->status[drone_current_mission_idx] = 1;
                        counter = 0.05;
                        first_check = 0;
                        
                        for(int j = this->drones_len -1;j>=0;j--){
                            if(this->drone_mission[drone_current_mission_idx][j]==1){
                                //change drones active li
                                if(first_check ==0){
                                    index_= j;
                                    first_check = 1;
                                    this->goal_status[j] = true;
                                }
                                else{
                                    drones_active[j]=false;
                                    Eigen::Vector2d ran;
                                    ran = this->point_genrator(0.5,0.001);
                                    std::cout << "deactivating drones, setting their destination to ";
                                    Eigen::Vector2d payload_dest(this->payload_location[drone_current_mission][0]+sign_data*ran(0), this->payload_location[drone_current_mission][1]+sign_data*ran(1));
                                    sign_data = sign_data * -1;
                                    this->swarm_config_tracker_->deactivate_drone(j,payload_dest);
                                }
                                counter += 0.05;
                            }
                        }

                        radii_[index_] = counter;
                        drones_active[index_] = true;
                        
                    }
                    
                    
                }
                else if (status[drone_current_mission_idx]==1){
                    check = 0;
                    final_x = this->payload_location[drone_current_mission][0];
                    final_y = this->payload_location[drone_current_mission][1];
                    //goto drop off
                    if(this->goal_status[i]==true){
                        this->goal[i](0) = final_x;
                        this->goal[i](1) = final_y; 
                        this->goal_status[i] = false;
                    }
                    //std::cout<<"given new goals\n";
                    float distance_=0.0;
                    for(int j=0; j<this->drones_len;j++){                        
                        if (drone_mission[drone_current_mission_idx][j]==1){
                            current_x = drone_states[j][0];
                            current_y = drone_states[j][1];
                            if(j == i && drones_active[i] == true){
                                previous_x = this->previous_state[j][0];
                                previous_y = this->previous_state[j][1]; 
                                distance_ = sqrt(pow((current_x-previous_x),2)+pow((current_y-previous_y),2));
                                int sum_=0;
                                for(int k = 0; k<this->drones_len;k++){
                                    if(this->drone_mission[drone_current_mission_idx][j]==1){
                                        sum_+=1; 
                                    }
                                }
                                distance_ = distance_*3;
                                this->total_distance[j]+=distance_;
                            }
                            
                            pay_(0) = current_x;
                            pay_(1) = current_y;
                            this->payloads_[drone_current_mission]->write_position(pay_);
                            distance = sqrt(pow((current_x-final_x),2)+pow((current_y-final_y),2));
                            std::cout<<"distance "<<distance<<std::endl;
                            if (distance<=0.1){
                                reach = 1;
                            }
                            else{
                                reach = 0;
                            } 
                            check = check + reach;
                        }
                    }
                    if(check != 0){
                        status[drone_current_mission_idx]=2; 
                        for(int j = 0;j<this->drones_len;j++){
                            if(drone_mission[drone_current_mission_idx][j]==1){ 
                                radii_[j] = 0.1;
                                this->goal_status[j] = true; 
                                //goals_[i](0) = this->payload_points[drone_current_mission][2];
                                //goals_[i](1) = this->payload_points[drone_current_mission][3];
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
        swarm_config_tracker_->write_drone_goals(goal);
        std::cout<<"wrote drone goals"<<" size : "<<goal.size()<<std::endl;
        swarm_config_tracker_->write_drone_active_vector(drones_active);
        std::cout<<"written drone active vector\n";
        swarm_config_tracker_->write_drone_radii(radii_);
        this->set_previous_state(drone_states);
        std::cout<<"written drone radii\n";



    }
}

#endif  //SWARM_SCHEDULER_H_

