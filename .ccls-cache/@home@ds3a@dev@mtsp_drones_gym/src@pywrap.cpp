#include "payload.hpp"
#include "workspace.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
// #include <pybind11/opencv.h>

namespace py = pybind11;
constexpr auto byref = py::return_value_policy::reference_internal;

PYBIND11_MODULE(mtsp_drones_gym, m) {
    m.doc() = "optional module docstring";

    py::class_<mtsp_drones_gym::Payload>(m, "Payload")
    .def(py::init<double, double, double, double, double>());
    // .def("run", &MyClass::run, py::call_guard<py::gil_scoped_release>())
    // .def_readonly("v_data", &MyClass::v_data, byref)
    // .def_readonly("v_gamma", &MyClass::v_gamma, byref)

    py::class_<mtsp_drones_gym::Workspace>(m, "Workspace")
    .def(py::init<bool>())
    .def("add_drone", &mtsp_drones_gym::Workspace::add_drone,
         py::arg("x"), py::arg("y"), py::arg("radius"), py::arg("capacity"))
    .def("add_payload", &mtsp_drones_gym::Workspace::add_payload,
         py::arg("x"), py::arg("y"), py::arg("mass"), py::arg("dest_x"), py::arg("dest_y"))
    .def("step", &mtsp_drones_gym::Workspace::step)
    .def("set_step_time", &mtsp_drones_gym::Workspace::set_step_time,
         py::arg("step_time"));
}
