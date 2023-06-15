#ifndef BASE_H_
#define BASE_H_

#include <Eigen/Dense>
#include <memory>
#include <variant>

namespace mtsp_drones_gym {
    #ifdef DOUBLE_PRECISION
        typedef Eigen::Vector2d vec;
    #endif

    #ifdef SINGLE_PRECISION
        typedef Eigen::Vector2f vec;
    #endif
}

#endif // BASE_H_
