#include <xbot2_interface/collision.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace XBot;
using namespace XBot::Collision;
namespace py = pybind11;
using rvp = py::return_value_policy;

auto check_collision = [](CollisionModel& self, bool include_env)
{
    std::vector<int> idx;
    bool ret = self.checkCollision(idx, include_env);
    return std::make_pair(ret, idx);
};

PYBIND11_MODULE(pyxbot2_collision, m) {

    py::class_<Collision::CollisionModel>(m, "CollisionModel")
        .def(py::init<ModelInterface::ConstPtr>(),
             py::arg("model"))
        .def("update", &CollisionModel::update)
        .def("checkCollision", check_collision,
             py::arg("include_env") = true)
        .def("checkSelfCollision", [&](CollisionModel& self){ return check_collision(self, false); })
        .def("computeDistance", py::overload_cast<bool, double>(&CollisionModel::computeDistance, py::const_),
             py::arg("include_env") = false, py::arg("threshold") = -1.0)
        .def("getDistanceJacobian", py::overload_cast<bool>(&CollisionModel::getDistanceJacobian, py::const_),
             py::arg("include_env") = false)
        .def("getCollisionPairs", &CollisionModel::getCollisionPairs,
             py::arg("include_env") = false)
        .def("getNormals", py::overload_cast<bool>(&CollisionModel::getNormals, py::const_),
             py::arg("include_env") = false)
        .def("getWitnessPoints", py::overload_cast<bool>(&CollisionModel::getWitnessPoints, py::const_),
             py::arg("include_env") = false)
        .def("getOrderedCollisionPairIndices", &CollisionModel::getOrderedCollisionPairIndices)
        ;

}
