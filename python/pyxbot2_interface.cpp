#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/common/utils.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace XBot;
namespace py = pybind11;
using rvp = py::return_value_policy;

PYBIND11_MODULE(pyxbot2_interface, m) {

    py::class_<ControlMode> controlMode(m, "ControlMode");

    controlMode
        .def_static("Position", &ControlMode::Position)
        .def_static("Velocity", &ControlMode::Velocity)
        .def_static("Effort", &ControlMode::Effort)
        .def_static("Stiffness", &ControlMode::Stiffness)
        .def_static("Damping", &ControlMode::Damping)
        .def_static("Impedance", &ControlMode::Impedance)
        .def_static("PosImpedance", &ControlMode::PosImpedance)
        .def("type", &ControlMode::type)
        .def(py::self + py::self)
        ;

    py::enum_<ControlMode::Type>(controlMode, "Type", py::arithmetic())
        .value("POSITION", ControlMode::Type::POSITION)
        .value("VELOCITY", ControlMode::Type::VELOCITY)
        .value("EFFORT", ControlMode::Type::EFFORT)
        .value("STIFFNESS", ControlMode::Type::STIFFNESS)
        .value("DAMPING", ControlMode::Type::DAMPING)
        .value("ACCELERATION", ControlMode::Type::ACCELERATION)
        .value("ALL", ControlMode::Type::ALL)
        .export_values();

    py::enum_<Sync>(m, "Sync")
        .value("LinkSide", Sync::LinkSide)
        .value("MotorSide", Sync::MotorSide)
        .export_values();

    m.def("computeOrientationError",
          py::overload_cast<const Eigen::Matrix3d&, const Eigen::Matrix3d&>(Utils::computeOrientationError));

    py::class_<XBotInterface>(m, "XBotInterface2")
        .def("getName",
             &XBotInterface::getName)
        .def("getJointNum",
             &XBotInterface::getJointNum)
        .def("getUrdfString",
             &XBotInterface::getUrdfString)
        .def("getSrdfString",
             &XBotInterface::getSrdfString)
        .def("getNq",
             &XBotInterface::getNq)
        .def("getNv",
             &XBotInterface::getNv)
        .def_property_readonly("nj",
                               &XBotInterface::getJointNum)
        .def_property_readonly("nq",
                               &XBotInterface::getNq)
        .def_property_readonly("nv",
                               &XBotInterface::getNv)
        .def_property_readonly("q0",
                               py::overload_cast<>(&XBotInterface::getNeutralQ, py::const_))
        .def_property_readonly("qminimal",
                               py::overload_cast<>(&XBotInterface::getJointPositionMinimal, py::const_))
        .def_property_readonly("qrand",
                               py::overload_cast<>(&XBotInterface::generateRandomQ, py::const_))
        .def_property_readonly("qlim",
                               py::overload_cast<>(&XBotInterface::getJointLimits, py::const_))
        .def_property_readonly("vlim",
                               py::overload_cast<>(&XBotInterface::getVelocityLimits, py::const_))
        .def_property_readonly("taulim",
                               py::overload_cast<>(&XBotInterface::getEffortLimits, py::const_))
        .def("minimalToPosition",
             py::overload_cast<VecConstRef>(&XBotInterface::minimalToPosition, py::const_))
        .def("positionToMinimal",
             py::overload_cast<VecConstRef>(&XBotInterface::positionToMinimal, py::const_))
        .def("getQNames",
             &XBotInterface::getQNames, rvp::reference_internal)
        .def("getVNames",
             &XBotInterface::getVNames, rvp::reference_internal)
        .def("getQIndexFromQName",
             &XBotInterface::getQIndexFromQName)
        .def("getVIndexFromVName",
             &XBotInterface::getVIndexFromVName)
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
        .def("getJointPositionMinimal",
             py::overload_cast<>(&XBotInterface::getJointPositionMinimal, py::const_))
        .def("getJointEffort",
             py::overload_cast<>(&XBotInterface::getJointEffort, py::const_))
        .def("getJointAcceleration",
             py::overload_cast<>(&XBotInterface::getJointAcceleration, py::const_))
        .def("getJointNames",
             py::overload_cast<>(&XBotInterface::getJointNames, py::const_))
        .def("getJoint",
             py::overload_cast<string_const_ref>(&XBotInterface::getJoint, py::const_))
        .def("getJoints",
             py::overload_cast<>(&XBotInterface::getJoints, py::const_), rvp::reference_internal)
        .def("getJoints",
             py::overload_cast<>(&XBotInterface::getJoints), rvp::reference_internal)
        .def("getJointId",
             py::overload_cast<string_const_ref>(&XBotInterface::getJointId, py::const_))
        .def("getJointInfo",
             py::overload_cast<string_const_ref>(&XBotInterface::getJointInfo, py::const_))
        .def("getJointInfo",
             py::overload_cast<int>(&XBotInterface::getJointInfo, py::const_))
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
             py::overload_cast<>(&XBotInterface::getForceTorque, py::const_), rvp::reference_internal)
        .def("getImu",
             py::overload_cast<>(&XBotInterface::getImu, py::const_), rvp::reference_internal)
        .def("getPose",
             py::overload_cast<string_const_ref>(&XBotInterface::getPose, py::const_))
        .def("getPose",
             py::overload_cast<string_const_ref,string_const_ref>(&XBotInterface::getPose, py::const_))
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
        .def("computeGravityCompensation",
             py::overload_cast<>(&XBotInterface::computeGravityCompensation, py::const_))
        .def("computeCentroidalMomentumMatrix",
             py::overload_cast<>(&XBotInterface::computeCentroidalMomentumMatrix, py::const_))
        .def("computeNonlinearTerm",
             py::overload_cast<>(&XBotInterface::computeNonlinearTerm, py::const_))
        .def("__str__", [](const XBotInterface& self)
             {
            std::stringstream ss;
            ss << "model '" << self.getName() <<
                "': nj = " << self.getJointNum() <<
                ", nq = " << self.getNq() <<
                ", nv = " << self.getNv() <<
                std::endl;
            return ss.str();
             })
        ;

    py::class_<ModelInterface, XBotInterface>(m, "ModelInterface2")
        .def(py::init(py::overload_cast<std::string, std::string>(&ModelInterface::getModel)),
             py::arg("urdf_string"), py::arg("model_type") = "pin")
        .def("getType",
             &ModelInterface::getType)
        .def("update",
             &ModelInterface::update)
        .def("getJoints",
             py::overload_cast<>(&ModelInterface::getJoints, py::const_), rvp::reference_internal)
        .def("getJoints",
             py::overload_cast<>(&ModelInterface::getJoints), rvp::reference_internal)
        .def("generateReducedModel",
             &ModelInterface::generateReducedModel)
        .def("setJointPosition",
             py::overload_cast<VecConstRef>(&ModelInterface::setJointPosition))
        .def("setJointPosition",
             py::overload_cast<const JointNameMap&>(&ModelInterface::setJointPosition))
        .def("setJointPositionMinimal",
             py::overload_cast<VecConstRef>(&ModelInterface::setJointPositionMinimal))
        .def("setJointVelocity",
             py::overload_cast<VecConstRef>(&ModelInterface::setJointVelocity))
        .def("setJointVelocity",
             py::overload_cast<const JointNameMap&>(&ModelInterface::setJointVelocity))
        .def("setJointAcceleration",
             py::overload_cast<VecConstRef>(&ModelInterface::setJointAcceleration))
        .def("setJointAcceleration",
             py::overload_cast<const JointNameMap&>(&ModelInterface::setJointAcceleration))
        .def("setJointEffort",
             py::overload_cast<VecConstRef>(&ModelInterface::setJointEffort))
        .def("setJointEffort",
             py::overload_cast<const JointNameMap&>(&ModelInterface::setJointEffort))
        .def("setJointLimits",
             &ModelInterface::setJointLimits)
        .def("setVelocityLimits",
             &ModelInterface::setVelocityLimits)
        .def("setEffortLimits",
             &ModelInterface::setEffortLimits)
        .def("setFloatingBasePose",
             &ModelInterface::setFloatingBasePose)
        .def("setFloatingBaseTwist",
             &ModelInterface::setFloatingBaseTwist)
        .def("addFixedLink",
             &ModelInterface::addFixedLink,
             py::arg("link_name"),
             py::arg("parent_name"),
             py::arg("mass") = 0,
             py::arg("inertia") =
             Eigen::Matrix3d::Zero().eval(),
             py::arg("pose") = Eigen::Affine3d::Identity())
        .def("updateFixedLink",
             &ModelInterface::updateFixedLink,
             py::arg("link_id"),
             py::arg("mass") = 0,
             py::arg("inertia") = Eigen::Matrix3d::Zero().eval(),
             py::arg("pose") = Eigen::Affine3d::Identity())
        .def("syncFrom",
             py::overload_cast<const RobotInterface&, ControlMode::Type, Sync>(&ModelInterface::syncFrom),
             py::arg("model"),
             py::arg("ctrl_mask") = ControlMode::ALL,
             py::arg("sync") = Sync::LinkSide)
        .def("syncFrom",
             py::overload_cast<const XBotInterface&, ControlMode::Type>(&ModelInterface::syncFrom),
             py::arg("robot"),
             py::arg("ctrl_mask") = ControlMode::ALL)
        .def_property("q",
                      py::overload_cast<>(&ModelInterface::getJointPosition, py::const_),
                      py::overload_cast<VecConstRef>(&ModelInterface::setJointPosition))
        .def_property("v",
                      py::overload_cast<>(&ModelInterface::getJointVelocity, py::const_),
                      py::overload_cast<VecConstRef>(&ModelInterface::setJointVelocity))
        .def_property("a",
                      py::overload_cast<>(&ModelInterface::getJointAcceleration, py::const_),
                      py::overload_cast<VecConstRef>(&ModelInterface::setJointAcceleration))
        .def_property("tau",
                      py::overload_cast<>(&ModelInterface::getJointEffort, py::const_),
                      py::overload_cast<VecConstRef>(&ModelInterface::setJointEffort))
        ;

    py::class_<RobotInterface, XBotInterface>(m, "RobotInterface2")
        .def(py::init(py::overload_cast<std::string, std::string, std::string>(
                 &RobotInterface::getRobot)),
             py::arg("urdf_string"), py::arg("robot_type"), py::arg("model_type") = "pin")
        .def("update",
             &RobotInterface::update)
        .def("sense",
             &RobotInterface::sense,
             py::arg("update_model") = true)
        .def("move",
             &RobotInterface::move)
        .def("model",
             &RobotInterface::model, rvp::reference_internal)
        .def("getJoints",
             py::overload_cast<>(&RobotInterface::getJoints, py::const_), rvp::reference_internal)
        .def("getJoints",
             py::overload_cast<>(&RobotInterface::getJoints), rvp::reference_internal)
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
             py::overload_cast<>(&RobotInterface::getControlMode, py::const_))
        .def("setControlMode",
             py::overload_cast<ControlMode::Type>(&RobotInterface::setControlMode))
        .def("setControlMode",
             py::overload_cast<CtrlModeVectorConstRef>(&RobotInterface::setControlMode))
        .def("setControlMode",
             py::overload_cast<const std::map<std::string, ControlMode::Type>&>(&RobotInterface::setControlMode))
        .def("getValidCommandMask",
             &RobotInterface::getValidCommandMask)        
        .def_property_readonly("q",
                      py::overload_cast<>(&RobotInterface::getJointPosition, py::const_))
        .def_property_readonly("v",
                      py::overload_cast<>(&RobotInterface::getJointVelocity, py::const_))
        .def_property_readonly("a",
                      py::overload_cast<>(&RobotInterface::getJointAcceleration, py::const_))
        .def_property_readonly("tau",
                               py::overload_cast<>(&RobotInterface::getJointEffort, py::const_))
        .def_property_readonly("k",
                               py::overload_cast<>(&RobotInterface::getStiffness, py::const_))
        .def_property_readonly("d",
                               py::overload_cast<>(&RobotInterface::getDamping, py::const_))
        ;

    py::class_<JointInfo>(m, "JointInfo")
        .def_readonly("id", &JointInfo::id)
        .def_readonly("iq", &JointInfo::iq)
        .def_readonly("iv", &JointInfo::iv)
        .def_readonly("nq", &JointInfo::nq)
        .def_readonly("nv", &JointInfo::nv)
        .def("iqv", &JointInfo::iqv)
        .def("nqv", &JointInfo::nqv)
        .def("inq", &JointInfo::inq)
        .def("inv", &JointInfo::inv)
        .def("__repr__", [](JointInfo& self)
             {
            std::stringstream ss;
            ss << "id = " << self.id <<
                "\tiq = " << self.iq <<
                "\tiv = " << self.iv <<
                "\tnq = " << self.nq <<
                "\tnv = " << self.nv;
            return ss.str();
        })
        ;



    py::class_<Sensor>(m, "Sensor")
        .def("getName",
             &Sensor::getName)
        .def("getTimestamp",
             &Sensor::getTimestamp)
        .def("isUpdated",
             &Sensor::isUpdated)
        ;

    py::class_<ImuSensor, Sensor>(m, "ImuSensor")
        .def("getAngularVelocity",
             py::overload_cast<>(&ImuSensor::getAngularVelocity, py::const_))
        .def("getLinearAcceleration",
             py::overload_cast<>(&ImuSensor::getLinearAcceleration, py::const_))
        .def("getOrientation",
             py::overload_cast<>(&ImuSensor::getOrientation, py::const_))
        ;

    py::class_<ForceTorqueSensor, Sensor>(m, "ForceTorqueSensor")
        .def("getWrench",
             py::overload_cast<>(&ForceTorqueSensor::getWrench, py::const_))
        ;

    py::class_<Joint, Joint::Ptr>(m, "Joint")
        .def("getJointInfo", &Joint::getJointInfo, py::return_value_policy::reference_internal)
        .def("getName", &Joint::getName)
        .def("getParentLink", &Joint::getParentLink)
        .def("getChildLink", &Joint::getChildLink)
        ;

    py::class_<ModelJoint, Joint, ModelJoint::Ptr>(m, "ModelJoint", py::multiple_inheritance());

    py::class_<RobotJoint, Joint, RobotJoint::Ptr>(m, "RobotJoint", py::multiple_inheritance());

}
