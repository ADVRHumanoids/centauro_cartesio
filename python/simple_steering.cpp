#include <centauro_cartesio/centauro_ankle_steering.h>
#include <centauro_cartesio/omnisteering_controller.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using XBot::Cartesian::Centauro::SimpleSteering;
using XBot::Cartesian::OmniSteeringController;

PYBIND11_MODULE(simple_steering, m)
{
    py::class_<XBot::Cartesian::Centauro::SimpleSteering>(m, "SimpleSteering")
        .def(py::init<XBot::ModelInterface::ConstPtr,
                      std::string,
                      std::vector<double>,
                      double>(),
             py::arg("model"),
             py::arg("wheel_name"),
             py::arg("hyst_thresholds") = std::vector<double>{0.0025, 0.01},
             py::arg("deadzone") = 0.01)
        .def("computeSteeringAngle", &SimpleSteering::computeSteeringAngle)
        .def("getWheelName", &SimpleSteering::getWheelName)
        .def("getSteeringJointName", &SimpleSteering::getSteeringJointName)
        ;

    py::class_<XBot::Cartesian::OmniSteeringController>(m, "OmniSteeringController")
        .def(py::init<XBot::ModelInterface::Ptr,
                      std::vector<std::string>,
                      std::vector<double>,
                      double,
                      double >(),
             py::arg("model"),
             py::arg("wheel_names"),
             py::arg("wheel_radius"),
             py::arg("dt"),
             py::arg("max_steering_speed"))
        .def("getSteeringJointNames", &OmniSteeringController::getSteeringJointNames)
        .def("getWheelJointNames", &OmniSteeringController::getWheelJointNames)
        .def("setBaseVelocity", &OmniSteeringController::setBaseVelocity)
        .def("update", &OmniSteeringController::update, py::arg("use_base_vel_from_model") = false)
        ;

}
