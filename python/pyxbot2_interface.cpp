#include <xbot2_interface/robotinterface2.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace XBot;
namespace py = pybind11;

PYBIND11_MODULE(pyxbot2_interface, m) {

    py::class_<XBotInterface>(m, "XBotInterface")
        .def("getName",
             &XBotInterface::getName)
        .def("getJointNum",
             &XBotInterface::getJointNum)
        .def("getNq",
             &XBotInterface::getNq)
        .def("getNv",
             &XBotInterface::getNv)
        .def("getJointLimits",
             py::overload_cast<>(&XBotInterface::getJointLimits, py::const_))
        .def("getVelocityLimits",
             py::overload_cast<>(&XBotInterface::getVelocityLimits, py::const_))
        .def("getEffortLimits",
             py::overload_cast<>(&XBotInterface::getEffortLimits, py::const_))
        .def("getRobotState",
             py::overload_cast<string_const_ref>(&XBotInterface::getRobotState, py::const_))
        .def("getJointPosition",
             py::overload_cast<>(&XBotInterface::getJointPosition, py::const_))
        .def("getJointEffort",
             py::overload_cast<>(&XBotInterface::getJointEffort, py::const_))
        .def("getJointAcceleration",
             py::overload_cast<>(&XBotInterface::getJointAcceleration, py::const_))
        .def("getJointNames",
             py::overload_cast<>(&XBotInterface::getJointNames, py::const_))
        .def("getJoint",
             py::overload_cast<string_const_ref>(&XBotInterface::getJoint, py::const_))
        .def("getJointId",
             py::overload_cast<string_const_ref>(&XBotInterface::getJointId, py::const_))
        .def("getJointInfo",
             py::overload_cast<string_const_ref>(&XBotInterface::getJointInfo, py::const_))
        .def("getFloatingBaseLink",
             py::overload_cast<>(&XBotInterface::getFloatingBaseLink, py::const_))
        .def("getMass",
             py::overload_cast<>(&XBotInterface::getMass, py::const_))
        .def("getNeutralQ",
             py::overload_cast<>(&XBotInterface::getNeutralQ, py::const_))
        .def("generateRandomQ",
             py::overload_cast<>(&XBotInterface::generateRandomQ, py::const_))
        .def("checkJointLimits",
             py::overload_cast<VecConstRef>(&XBotInterface::checkJointLimits, py::const_))
        .def("sum",
             py::overload_cast<VecConstRef, VecConstRef>(&XBotInterface::sum, py::const_))
        .def("difference",
             py::overload_cast<VecConstRef, VecConstRef>(&XBotInterface::difference, py::const_))
        .def("getForceTorque",
             py::overload_cast<>(&XBotInterface::getForceTorque, py::const_),
             py::return_value_policy::reference)
        .def("getImu",
             py::overload_cast<>(&XBotInterface::getImu, py::const_),
             py::return_value_policy::reference)
        .def("getPose",
             py::overload_cast<string_const_ref>(&XBotInterface::getPose, py::const_))
        .def("getAccelerationTwist",
             py::overload_cast<string_const_ref>(&XBotInterface::getAccelerationTwist, py::const_))
        .def("getVelocityTwist",
             py::overload_cast<string_const_ref>(&XBotInterface::getVelocityTwist, py::const_))
        .def("getJdotTimesV",
             py::overload_cast<string_const_ref>(&XBotInterface::getJdotTimesV, py::const_))
        .def("getCOM",
             py::overload_cast<>(&XBotInterface::getCOM, py::const_))
        .def("getCOMJacobian",
             py::overload_cast<>(&XBotInterface::getCOMJacobian, py::const_))
        .def("getRelativeVelocityTwist",
             py::overload_cast<string_const_ref, string_const_ref>(&XBotInterface::getRelativeVelocityTwist, py::const_))
        .def("getRelativeAccelerationTwist",
             py::overload_cast<string_const_ref, string_const_ref>(&XBotInterface::getRelativeAccelerationTwist, py::const_))
        .def("getRelativeJdotTimesV",
             py::overload_cast<string_const_ref, string_const_ref>(&XBotInterface::getRelativeJdotTimesV, py::const_))
        .def("getJacobian",
             py::overload_cast<string_const_ref>(&XBotInterface::getJacobian, py::const_))
        .def("getRelativeJacobian",
             py::overload_cast<string_const_ref, string_const_ref>(&XBotInterface::getRelativeJacobian, py::const_))
        .def("computeInverseDynamics",
             py::overload_cast<>(&XBotInterface::computeInverseDynamics, py::const_))
        .def("computeForwardDynamics",
             py::overload_cast<>(&XBotInterface::computeForwardDynamics, py::const_))
        .def("computeInertiaMatrix",
             py::overload_cast<>(&XBotInterface::computeInertiaMatrix, py::const_))
        .def("computeInertiaInverse",
             py::overload_cast<>(&XBotInterface::computeInertiaInverse, py::const_))
        ;

    py::class_<ModelInterface, XBotInterface>(m, "ModelInterface")
        .def(py::init(py::overload_cast<std::string, std::string>(&ModelInterface::getModel)),
             py::arg("urdf_string"), py::arg("model_type") = "pin")
        .def("getType",
             &ModelInterface::getType)
        .def("update",
             &ModelInterface::update)
        .def("generateReducedModel",
             &ModelInterface::generateReducedModel)
        .def("setJointPosition",
             &ModelInterface::setJointPosition)
        .def("setJointVelocity",
             &ModelInterface::setJointVelocity)
        .def("setJointAcceleration",
             &ModelInterface::setJointAcceleration)
        .def("setJointEffort",
             &ModelInterface::setJointEffort)
        .def("setJointLimits",
             &ModelInterface::setJointLimits)
        .def("setVelocityLimits",
             &ModelInterface::setVelocityLimits)
        .def("setEffortLimits",
             &ModelInterface::setEffortLimits)
        .def("setFloatingBasePose",
             &ModelInterface::setFloatingBasePose)
        // .def("setFloatingBaseState", &ModelInterface::setFloatingBaseState)
        .def("setFloatingBaseTwist",
             &ModelInterface::setFloatingBaseTwist)
        ;

    py::class_<RobotInterface, XBotInterface>(m, "RobotInterface")
        .def(py::init(py::overload_cast<std::string, std::string, std::string>(
                 &RobotInterface::getRobot)),
             py::arg("urdf_string"), py::arg("robot_type"), py::arg("model_type") = "pin")
        .def("update",
             &RobotInterface::update)
        .def("sense",
             &RobotInterface::sense)
        .def("move",
             &RobotInterface::move)
        .def("getMotorPosition",
             py::overload_cast<>(&RobotInterface::getMotorPosition, py::const_))
        .def("getMotorVelocity",
             py::overload_cast<>(&RobotInterface::getMotorVelocity, py::const_))
        .def("getPositionReference",
             py::overload_cast<>(&RobotInterface::getPositionReference, py::const_))
        .def("getVelocityReference",
             py::overload_cast<>(&RobotInterface::getVelocityReference, py::const_))
        .def("getEffortReference",
             py::overload_cast<>(&RobotInterface::getEffortReference, py::const_))
        .def("getStiffness",
             py::overload_cast<>(&RobotInterface::getStiffness, py::const_))
        .def("getDamping",
             py::overload_cast<>(&RobotInterface::getDamping, py::const_))
        .def("setPositionReference",
             &RobotInterface::setPositionReference)
        .def("setVelocityReference",
             &RobotInterface::setVelocityReference)
        .def("setEffortReference",
             &RobotInterface::setEffortReference)
        .def("setStiffness",
             &RobotInterface::setStiffness)
        .def("setDamping",
             &RobotInterface::setDamping)
        .def("getControlMode",
             &RobotInterface::getControlMode)
        .def("setControlMode",
             &RobotInterface::setControlMode)
        .def("getValidCommandMask",
             &RobotInterface::getValidCommandMask)
        ;

    py::class_<JointInfo>(m, "JointInfo")
        .def_readonly("id", &JointInfo::id)
        .def_readonly("iq", &JointInfo::iq)
        .def_readonly("iv", &JointInfo::iv)
        .def_readonly("nq", &JointInfo::nq)
        .def_readonly("nv", &JointInfo::nv)
        .def("__repr__", [](JointInfo& self)
             {
            std::stringstream ss;
            ss << "id = " << self.id <<
                " iq = " << self.iq <<
                " iv = " << self.iv <<
                " nq = " << self.nq <<
                " nv = " << self.nv;
            return ss.str();
        })
        ;


}
