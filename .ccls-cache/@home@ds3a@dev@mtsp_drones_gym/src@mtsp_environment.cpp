#include "payload.hpp"
#include "swarm_planner.hpp"
#include "workspace.hpp"
#include <chrono>
#include <thread>

int main() {
    mtsp_drones_gym::Workspace ws(true);
    ws.add_drone(0, 0, 0.1, 1);
    ws.add_drone(-1.5, 0, 0.1, 1);
    ws.set_step_time(0.015);

    mtsp_drones_gym::Move dronea, droneb;
    dronea = (mtsp_drones_gym::Move) {.x = 1, .y = 0};
    droneb = (mtsp_drones_gym::Move) {.x = 0, .y = 0};
    ws.set_actions(std::vector<mtsp_drones_gym::DroneAction>{dronea, droneb});

    for (int i=0; i<100; i++) {
        ws.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(15));

    }

    return 0;
}
