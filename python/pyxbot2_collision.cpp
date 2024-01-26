#include <xbot2_interface/collision.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace XBot;
namespace py = pybind11;
using rvp = py::return_value_policy;

PYBIND11_MODULE(pyxbot2_collision, m) {

    py::class_<Collision::CollisionModel>(m, "CollisionModel")
        ;

}
