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

    py::module_ sm = m.def_submodule("shape");

    py::class_<Shape::Sphere>(sm, "Sphere")
        .def(py::init<>())
        .def_readwrite("radius", &Shape::Sphere::radius);

    py::class_<Shape::Capsule>(sm, "Capsule")
        .def(py::init<>())
        .def_readwrite("radius", &Shape::Capsule::radius)
        .def_readwrite("length", &Shape::Capsule::length);

    py::class_<Shape::Halfspace>(sm, "Halfspace")
        .def(py::init<>())
        .def_readwrite("normal", &Shape::Halfspace::normal)
        .def_readwrite("d", &Shape::Halfspace::d);

    py::class_<Shape::Box>(sm, "Box")
        .def(py::init<>())
        .def_readwrite("size", &Shape::Box::size);

    py::class_<Shape::Cylinder>(sm, "Cylinder")
        .def(py::init<>())
        .def_readwrite("radius", &Shape::Cylinder::radius)
        .def_readwrite("length", &Shape::Cylinder::length);

    py::class_<Shape::Mesh>(sm, "Mesh")
        .def(py::init<>())
        .def_readwrite("filepath", &Shape::Mesh::filepath)
        .def_readwrite("scale", &Shape::Mesh::scale)
        .def_readwrite("convex", &Shape::Mesh::convex);

    py::class_<Shape::Octree>(sm, "Octree")
        .def(py::init<>()) // binding std::any content is skipped
        .def_readwrite("data", &Shape::Octree::data);

    py::class_<Shape::HeightMap>(sm, "HeightMap")
        .def(py::init<>())
        .def_readwrite("dim_x", &Shape::HeightMap::dim_x)
        .def_readwrite("dim_y", &Shape::HeightMap::dim_y)
        .def_property("height",
            [](const Shape::HeightMap &self) { return *self.height; },
            [](Shape::HeightMap &self, const Eigen::MatrixXf &mat) {
                self.height = std::make_shared<Eigen::MatrixXf>(mat);});

    py::class_<Shape::MeshRaw>(sm, "MeshRaw")
        .def(py::init<>())
        .def_readwrite("vertices", &Shape::MeshRaw::vertices)
        .def_readwrite("triangles", &Shape::MeshRaw::triangles)
        .def_readwrite("convex", &Shape::MeshRaw::convex);

    // // Shape::Variant as a Python type (using pybind11's variant_caster)
    // py::class_<Shape>(m, "Shape"); // Tag class for grouping
    // using ShapeVariant = Shape::Variant;

    // py::class_<ShapeVariant>(m, "ShapeVariant");

    py::class_<CollisionModel::Options>(m, "CollisionModelOptions")
        .def(py::init<>())
        .def_readwrite("assume_convex_meshes", &CollisionModel::Options::assume_convex_meshes);

    py::class_<Collision::CollisionModel, Collision::CollisionModel::Ptr> cm(m, "CollisionModel");
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



}
