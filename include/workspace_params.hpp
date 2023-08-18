#ifndef WORKSPACE_PARAMS_H_
#define WORKSPACE_PARAMS_H_

#define DOUBLE_PRECISION

#include "base.hpp"

namespace mtsp_drone_delivery {
    class WorkspaceParams {
        private:
        double length;
        double width;

        std::vector<Eigen::Vector4d> drone_states;
        std::vector<Eigen::Vector4d> payload_states;
    };
}


#endif // WORKSPACE_PARAMS_H_
