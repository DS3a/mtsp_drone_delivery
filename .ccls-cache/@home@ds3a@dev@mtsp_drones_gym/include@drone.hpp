#ifndef DRONE_H_
#define DRONE_H_

#define DOUBLE_PRECISION

#include "base.hpp"

namespace mtsp_drones_gym {

    /*
     * Actions for the drones
     * */

    struct Move {
        double x;
        double y;
    };
    struct Pick {};
    struct Drop {};
    struct Attach {};
    struct Detach {};

    using DroneAction = std::variant<
      Move,
      Drop,
      Attach,
      Detach>;


    // The drone class
    class Drone {
    private:
        vec initial_position;

        vec position;

        vec velocity;

        bool is_carrying_payload_ = false;
        std::shared_ptr<Payload> payload_;

    public:

        // this is for graphical representation as well as obstacle avoidance calcuations
        double radius_ = 0.1; // in meters

        double capacity_ = 1; // the mass that can be carried

        Drone(double x, double y, double radius, double capacity);

        const vec* get_position();

        const vec* get_velocity();

        void set_velocity(vec velocity_);

        Eigen::Vector4d step(double step_time);

        bool is_carrying_payload();

        Eigen::Vector4d get_state();

        friend class Workspace;
    };

    Drone::Drone(double x, double y, double radius, double capacity) {
        this->initial_position = vec(x, y);
        this->position = vec(x, y);
        this->velocity = vec(1, -1);
        this->radius_ = radius;
        this->capacity_ = capacity;
    }

    Eigen::Vector4d Drone::get_state() {
        Eigen::Vector4d state;
        state << this->position, this->velocity;
        return state;
    }

    const vec* Drone::get_position() {
        return &this->position;
    }

    const vec* Drone::get_velocity() {
        return &this->velocity;
    }

    void Drone::set_velocity(vec velocity_) {
        this->velocity = velocity_;
    }

    Eigen::Vector4d Drone::step(double step_time) {
        // std::cout << "the current position is " << this->position;
        this->position += this->velocity * step_time;
        // std:: cout << "\nthe new position is " << this->position << std::endl << std::endl;

        if (this->is_carrying_payload_) {
            this->payload_->position = this->position;
        }

        return this->get_state();
    }

    bool Drone::is_carrying_payload() { return this->is_carrying_payload_; }
}

#endif // DRONE_H_
