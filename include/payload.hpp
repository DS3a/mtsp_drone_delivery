#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#define DOUBLE_PRECISION

#include "base.hpp"
#include <iostream>


namespace mtsp_drones_gym {
    class Payload {
    private:
        vec initial_position;

        // position in meters
        vec position;

        // velocity in meters/sec
        vec velocity;

        // mass
        float mass_;

        // this is for graphical visualization
        float radius_=0.05;

        // to be set to true if a drone picks it up
        bool is_carried = false;

        // the destination of the payload
        vec destination;

    public:
        Payload(double x, double y, double mass, double dest_x, double dest_y);

        Eigen::Vector4d get_state();

        void write_position(Eigen::Vector2d new_position);

        Eigen::Vector4d get_start_and_dest();

        friend class Drone;
        friend class Workspace;
    };

    void Payload::write_position(Eigen::Vector2d new_position) {
        this->position = new_position;
    }

    Payload::Payload(double x, double y, double mass,
                     double dest_x, double dest_y) {
        this->initial_position = vec(x, y);
        this->position = vec(x, y);
        this->mass_ = mass;
        this->radius_ = mass*0.05;
        this->destination = vec(dest_x, dest_y);

        // std::cout << "initializing payload with mass " << this->mass_ << "\nand position " << this->position << std::endl;
    }

    Eigen::Vector4d Payload::get_state() {
        Eigen::Vector4d state;
        state << this->position, this->velocity;
        return state;
    }

    Eigen::Vector4d Payload::get_start_and_dest() {
        Eigen::Vector4d start_and_dest;
        start_and_dest << this->initial_position, this->destination;
        return start_and_dest;
    }
}


#endif // PAYLOAD_H_
