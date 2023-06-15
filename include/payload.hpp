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
        float radius_=0.5;

        // to be set to true if a drone picks it up
        bool is_carried = false;

        // the destination of the payload
        vec destination;

    public:
        Payload(double x, double y, double mass, double dest_x, double dest_y);

        friend class Drone;
        friend class Workspace;
    };

    Payload::Payload(double x, double y, double mass,
                     double dest_x, double dest_y) {
        this->initial_position = vec(x, y);
        this->position = vec(x, y);
        this->mass_ = mass;
        this->destination = vec(dest_x, dest_y);

        // std::cout << "initializing payload with mass " << this->mass_ << "\nand position " << this->position << std::endl;
    }

}


#endif // PAYLOAD_H_
