#ifndef SWARM_SCHEDULER_H_
#define SWARM_SCHEDULER_H_


#include<stdio.h>
#include<vector>
#include<cmath>
#include <map>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include "workspace.hpp"

namespace swarm_scheduler{
    class SwarmScheduler{
        private:
            int mission_len; //payload len
            std::vector<int> mission_idx; //payload id
            int drones_len; //no of drones
            std::vector<std::vector<int>> drone_mission; //list of list of matrix
            std::vector <int> status; 
            std::map<int, std::vector<int>> drone_planner;
            std::map<int, std::vector<int>> mission_logger;
            std::vector<int> drones_list;
            std::vector<Eigen::Vector4d> drone_states;
            std::vector<Eigen::Vector4d> payload_points;
            std::vector<Eigen::Vector2d> goals_;
            std::vector<double> radii_;
            std::vector<bool> drones_active;
            std::vector<Eigen::Vector2d> payload_dict; // payload_id, drone_id


        public:

            void intitization(std::vector<std::vector<int>> mission_);
            //getter functions
            int get_mission_len();
            std::vector<int> getmission_idx();
            int getdrone_len();
            std::vector<std::vector<int>> getdrone_mission();
            std::vector<int>getstatus(void);
            std::map<int,std::vector<int>> getdrone_planner();
            std::map<int,std::vector<int>> getmission_logger();
            std::vector<int> getdrones();

            //setter functions
            void setmissions_len(int x);
            void setmisission_idx(int x);
            void setdrones_len(int x);
            void setdrones_missions(const std::vector<std::vector<int>> x);
            void setdrones(void);
            void createstatus_list();
            void radius_create();

            void missions();
            void mission_check(void);

            friend class SwarmConfigTracker

    };
} // namespace swarm_scheduler
#endif  //SWARM_SCHEDULER_H_
