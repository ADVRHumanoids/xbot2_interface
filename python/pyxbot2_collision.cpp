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



    py::class_<Collision::CollisionModel> cm(m, "CollisionModel");
    cm
        .def(py::init<ModelInterface::ConstPtr, CollisionModel::Options>(),
             py::arg("model"),
             py::arg("options") = CollisionModel::Options())
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
        .def("addCollisionShape",
             &CollisionModel::addCollisionShape,
             py::arg("name"),
             py::arg("link"),
             py::arg("shape"),
             py::arg("link_T_shape") = Eigen::Affine3d::Identity(),
             py::arg("disabled_collisions") = std::vector<std::string>{})
        ;

    py::class_<CollisionModel::Options>(cm, "Options")
        .def_readwrite("assume_convex_meshes", &CollisionModel::Options::assume_convex_meshes)
        ;


    // Shape sub-struct bindings
    py::class_<Shape::Sphere>(m, "Sphere")
        .def(py::init<double>())
        .def_readwrite("radius", &Shape::Sphere::radius);

    py::class_<Shape::Capsule>(m, "Capsule")
        .def(py::init<>())
        .def_readwrite("radius", &Shape::Capsule::radius)
        .def_readwrite("length", &Shape::Capsule::length);

    py::class_<Shape::Box>(m, "Box")
        .def(py::init<>())
        .def_readwrite("size", &Shape::Box::size);

    py::class_<Shape::Cylinder>(m, "Cylinder")
        .def(py::init<>())
        .def_readwrite("radius", &Shape::Cylinder::radius)
        .def_readwrite("length", &Shape::Cylinder::length);

    py::class_<Shape::Halfspace>(m, "Halfspace")
        .def(py::init<>())
        .def_readwrite("normal", &Shape::Halfspace::normal)
        .def_readwrite("d", &Shape::Halfspace::d);

    py::class_<Shape::Mesh>(m, "Mesh")
        .def(py::init<>())
        .def_readwrite("filepath", &Shape::Mesh::filepath)
        .def_readwrite("scale", &Shape::Mesh::scale)
        .def_readwrite("convex", &Shape::Mesh::convex);

    py::class_<Shape::Octree>(m, "Octree")
        .def(py::init<>())
        .def_readwrite("data", &Shape::Octree::data);

    // py::class_<Shape::HeightMap>(m, "HeightMap")
    //     .def(py::init<>())
    //     .def_readwrite("dim_x", &Shape::HeightMap::dim_x)
    //     .def_readwrite("dim_y", &Shape::HeightMap::dim_y)
    //     .def_readwrite("height", &Shape::HeightMap::height);

    // Shape::Variant as a Python type (using pybind11's variant_caster)
    py::class_<Shape>(m, "Shape"); // Tag class for grouping
    using ShapeVariant = Shape::Variant;

    py::class_<ShapeVariant>(m, "ShapeVariant");

    // Register variant_caster for Shape::Variant
//     PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
//     PYBIND11_VARIANT_CASTER(ShapeVariant, Shape::Sphere, Shape::Capsule, Shape::Box, Shape::Cylinder, Shape::Halfspace, Shape::Mesh, Shape::Octree, Shape::HeightMap);
}
