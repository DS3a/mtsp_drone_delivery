cmake_minimum_required(VERSION 3.10)

project(mtsp_drone_delivery)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PYBIND11_PYTHON_VERSION 3.6)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")

find_package(OpenCV REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)


# pybind11_add_module(${PROJECT_NAME} src/pywrap.cpp)
add_executable(${PROJECT_NAME} src/mtsp_environment.cpp)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  )

# target_compile_definitions(${PROJECT_NAME} PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OpenCV_LIBS}
  ${PYBIND11_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
  )
