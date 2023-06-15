#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "base.hpp"
#include "payload.hpp"
#include "drone.hpp"
#include <opencv2/opencv.hpp>
#include "math.h"

// double get_dist(const mtsp_drones_gym::vec* point1, const mtsp_drones_gym::vec* point2) {
//     return (double)sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
// }

namespace mtsp_drones_gym {
    class Workspace {
    private:
        // in meters
        double height;
        double width;

        double step_time_;
        std::vector<mtsp_drones_gym::Drone> drones;
        std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads;

        bool render=false;

        // the distance between the center points of drones below which they will be assumed to be colliding
        double collision_threshold = 0.15;

        // this function checks for collisions and returns a tuple. The first element is whether there was a collision or not
        // and the second is the graph of all drones present in the workspace containing info about which drones are colliding with which ones
        std::tuple<bool, std::vector<std::vector<bool>>> check_collisions();



      public:
        Workspace();

        // this function sets all the drones velocities
        void set_actions();

        // this function steps the simulator forward with the velocities of all elements
        void step();

        void set_step_time(double step_time) {
            this->step_time_ = step_time;
        }

        // reset the environment
        void reset();

        void add_drone(double x, double y, double radius, double capacity);

        void add_payload(double x, double y, double mass, double dest_x, double dest_y);
    };

    Workspace::Workspace() {
        std::cout << "initializing workspace\n";
    }

    void Workspace::step() {

    }

    std::tuple<bool, std::vector<std::vector<bool>>> Workspace::check_collisions() {
        bool collision=false;
        std::vector<std::vector<bool>> collision_graph(this->drones.size(), std::vector<bool>(this->drones.size(), false));
        for (int i=0; i < this->drones.size(); i++) {
            for (int j=i+1; j < this->drones.size(); j++) {
                if (i != j) {
                    double p1x = (*this->drones[i].get_position())[0];
                    double p1y = (*this->drones[i].get_position())[1];
                    double p2x = (*this->drones[j].get_position())[0];
                    double p2y = (*this->drones[j].get_position())[1];
                    double dist = sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2));

                    // subtracting the radii of the drones from the distance to consider edge-to-edge distance
                    dist -= (this->drones[i].radius_ + this->drones[j].radius_);
                    if (dist < this->collision_threshold) {
                        collision_graph[i][j] = true;
                        collision_graph[j][i] = true;
                        collision = true;
                    }
                }
            }
        }

        return std::make_tuple(collision, collision_graph);
    }

    void Workspace::add_drone(double x, double y,
                              double radius, double capacity) {
        std::cout << "adding a drone at position " << x << ", " << y << std::endl;
        this->drones.push_back(Drone(x, y, radius, capacity));
    }

    void Workspace::add_payload(double x, double y, double mass,
                                double dest_x, double dest_y) {
        std::cout << "adding a payload at " << x << ", " << y << std::endl;
        this->payloads.push_back(std::make_shared<Payload>(Payload(x, y, mass, dest_x, dest_y)));
    }
}


#endif // WORKSPACE_H_
