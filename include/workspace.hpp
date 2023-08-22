#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "base.hpp"
#include "payload.hpp"
#include "drone.hpp"
#include "swarm_planner.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "math.h"



namespace mtsp_drones_gym {
    class Workspace {
    private:
        // in meters
        double length = 4.5; // along x axis
        double width = 4; // along y axis

        vec origin = vec(2.25, 2.00);

        double step_time_;
        std::vector<mtsp_drones_gym::Drone> drones;
        std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> payloads;
        std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker_;

        bool render_ = false;
        double render_resolution = 0.005;
        cv::Mat frame;

        // the distance between the center points of drones below which they will be assumed to be colliding
        double collision_threshold = 0.15;

        // this function checks for collisions and returns a tuple. The first element is whether there was a collision or not
        // and the second is the graph of all drones present in the workspace containing info about which drones are colliding with which ones
        std::tuple<bool, std::vector<std::vector<bool>>> check_collisions();

        // these functions are for converting image points to the irl points and the irl points to the image points. This would be used for the costmap generation
        vec img_to_irl(const vec* img_point);

        vec irl_to_img(const vec* irl_point);

        std::vector<Move> actions_;

        void update_window();

        std::vector<vec> get_bounds();

    public:
        Workspace(bool render);

        void draw_paths(std::vector<std::vector<vec>> paths, std::vector<bool> paths_found);

        // this function sets all the drones velocities
        void set_actions(std::vector<Move>);

        // this function steps the simulator forward with the velocities of all elements
        std::tuple<bool, std::vector<Eigen::Vector4d>, std::vector<Eigen::Vector4d>> step();

        void set_step_time(double step_time) {
            this->step_time_ = step_time;
        }

        // reset the environment
        void reset();

        void add_drone(double x, double y, double radius, double capacity);

        void add_payload(double x, double y, double mass, double dest_x, double dest_y);

        std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> get_payloads();

        void set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker);

    };

    // END Workspace declaration
    //
    // BEGIN Workspace function definitions

    Workspace::Workspace(bool render) {
        this->render_ = render;
        std::cout << "initializing workspace\n";
        if (this->render_) {
            this->frame = cv::Mat(this->length/this->render_resolution, this->width/this->render_resolution, CV_8UC3, cv::Scalar(255, 255, 255));
            cv::namedWindow("mtsp drones");
        }
    }

    void Workspace::set_swarm_config_tracker(std::shared_ptr<swarm_planner::SwarmConfigTracker> swarm_config_tracker) {
        this->swarm_config_tracker_ = swarm_config_tracker;
    }

    std::vector<vec> Workspace::get_bounds() {
        return std::vector<vec> {vec(length-origin[0], -length+origin[0]), vec(width-origin[0], -width+origin[0])};
    }

    void Workspace::set_actions(std::vector<Move> actions) {
        this->actions_ = actions;
    }

    void Workspace::update_window() {
        cv::imshow("mtsp drones", this->frame);
        if ((char)cv::waitKey(25) == 27) {
            exit(0);
        }
    }

    void Workspace::draw_paths(std::vector<std::vector<vec>> paths, std::vector<bool> paths_found) {
        std::cout << "drawing paths\n";
        int i=0;
        for (auto path: paths) {
            std::cout << "inside the for loop\n";
            if (paths_found[i++]) {
                for (int i=0; i < path.size() - 1; i++) {
                    vec img_coords_0 = this->irl_to_img(&path[i]);
                    vec img_coords_1 = this->irl_to_img(&path[i+1]);
                    cv::Point start_point(img_coords_0[0], img_coords_0[1]);
                    cv::Point end_point(img_coords_1[0], img_coords_1[1]);
                    cv::line(this->frame, start_point, end_point, cv::Scalar(0, 0, 255), 2);
                    std::cout << "drawing a new line\n";
                }
            }
        }
        std::cout << "done drawing lines\n";

        this->update_window();
        std::cout << "donen updating window\n";
    }

    // TODO
    std::tuple<bool, std::vector<Eigen::Vector4d>, std::vector<Eigen::Vector4d>> Workspace::step() {
        std::vector<Eigen::Vector4d> drone_states;
        std::vector<Eigen::Vector4d> payload_states;


        if (this->actions_.size() != this->drones.size()) {
            return std::make_tuple(false, drone_states, payload_states);
        } else {
            for (int i=0; i < this->actions_.size(); i++) {
                // std::visit(overloaded {
                    // [this, i](Move m) {this->drones[i].set_velocity(vec(m.x, m.y));},
                    // [this, i](Pick p) {this->drones[i].set_velocity(vec(0, 0));},
                    // [this, i](Drop d) {this->drones[i].set_velocity(vec(0, 0));},
                    // [this, i](Attach a) {this->drones[i].set_velocity(vec(0, 0));},
                    // [this, i](Detach d) {this->drones[i].set_velocity(vec(0, 0));}
                // }, this->actions_[i]);
                this->drones[i].set_velocity(vec(this->actions_[i].x, this->actions_[i].y));
            }
        }

        for (Drone& drone: this->drones) {
            drone_states.push_back(drone.step(this->step_time_));
        }

        this->check_collisions();

        if (this->render_) {
            std::cout << "getting drone radii\n";
            std::vector<double> drone_radii = this->swarm_config_tracker_->read_drone_radii();
            std::vector<bool> drone_active = this->swarm_config_tracker_->read_drone_active();
            // std::cout << "got drone radii " << drone_radii[0] << std::endl;
            this->frame = cv::Mat(this->length/this->render_resolution, this->width/this->render_resolution, CV_8UC3, cv::Scalar(255, 255, 255));
            int i = 0;
            for (Drone& drone: this->drones) {
                if (drone_active[i]) {
                    cv::Point center;
                    vec img_coords = this->irl_to_img(drone.get_position());
                    center.x = img_coords[0];
                    center.y = img_coords[1];
                    cv::circle(this->frame, center, drone_radii[i++]/this->render_resolution, cv::Scalar(230, 130, 155), -1);
                }
            }

            int payload_id = 1;

           for (std::shared_ptr<Payload> payload: this->payloads) {
               cv::Point center;
               vec img_coords = this->irl_to_img(&payload->position);
               center.x = img_coords[0];
               center.y = img_coords[1];

               if ((payload->position - payload->destination).norm() >= 0.1) {
                   vec img_coords = this->irl_to_img(&payload->position);
                   center.x = img_coords[0];
                   center.y = img_coords[1];
                   cv::circle(this->frame, center,
                              payload->radius_ / this->render_resolution,
                              cv::Scalar(0, 255, 0), -1);

                   cv::putText(this->frame, std::to_string(payload_id), center,
                               2, 1.0, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

                   img_coords = this->irl_to_img(&payload->destination);
                   center.x = img_coords[0];
                   center.y = img_coords[1];
                   cv::circle(this->frame, center,
                              payload->radius_ / this->render_resolution,
                              cv::Scalar(155, 155, 155), -1);
                   cv::putText(this->frame,
                               std::to_string(payload_id++) + " dest", center, 2,
                               1.0, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
               } else {
                   cv::circle(this->frame, center,
                              payload->radius_ / this->render_resolution,
                              cv::Scalar(0, 155, 155), -1);
                   cv::putText(this->frame,
                               std::to_string(payload_id++) + " delivered", center, 2,
                               1.0, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

               }
           }

            // this->update_window();
            return std::make_tuple(true, drone_states, payload_states);
        }

    }

    vec Workspace::img_to_irl(const vec* img_point) {
        double um = (*img_point)[0] * this->render_resolution;
        double vm = (*img_point)[1] * this->render_resolution;

        double y = this->origin[1] - um;
        double x = this->origin[0] - vm;
        return vec(x, y);
    }

    vec Workspace::irl_to_img(const vec* irl_point) {
        int v = (this->origin[0] - (*irl_point)[0])/this->render_resolution;
        int u = (this->origin[1] - (*irl_point)[1])/this->render_resolution;

        return vec(u, v);
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

    std::vector<std::shared_ptr<mtsp_drones_gym::Payload>> Workspace::get_payloads() {
        // Returns std::vector<(start_x, start_y, end_x, end_y)>
        // std::vector<Eigen::Vector4d> payload_states;

        // for (int i=0; i < this->payloads.size(); i++) {
            // payload_states.push_back(this->payloads[i]->get_start_and_dest());
        // }

        return this->payloads;
    }
}


#endif // WORKSPACE_H_
