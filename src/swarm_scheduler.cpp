
#include<stdio.h>
#include<vector>
#include<cmath>

class mission{
    private:
    int mission_len;
    std::vector<int> mission_idx;
    int drones_len;
    std::vector<std::vector<int>> drone_mission;
    std::vector <int> status;
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

    void createstatus_list(){
        status.clear();
        for(int i=0 ; i<mission_len; i++){
            status.push_back(0);
            //status = 0 not started
            //status = 1 started 
            //status = 2 queued
            //status = 3 compeled
        }
    }

};
