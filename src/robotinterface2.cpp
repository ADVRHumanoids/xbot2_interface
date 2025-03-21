#include "impl/xbotinterface2.hxx"
#include "impl/robotinterface2.hxx"

#include "impl/load_object.h"

#include <iostream>

using namespace XBot;

bool RobotInterface::sense(bool update_model)
{
    for(auto& item : XBotInterface::impl->_sensor_map)
    {
        item.second->clear();
    }

    if(!sense_impl())
    {
        return false;
    }

    if(update_model)
    {
        r_impl->_model->update();
    }

    return true;
}

bool RobotInterface::move()
{
    return move_impl();
}

RobotInterface::UniquePtr RobotInterface::getRobot(ConfigOptions opt)
{
    auto mdl = ModelInterface::getModel(opt);

    std::string robot_type = "ros2";

    opt.get_parameter("robot_type", robot_type);

    bool ignore_type_from_env = false;

    opt.get_parameter("ignore_type_from_env", ignore_type_from_env);

    const char * robot_type_env = getenv("XBOT2IFC_ROBOT_TYPE");

    if(robot_type_env && !ignore_type_from_env)
    {
        robot_type = robot_type_env;
    }

    auto rob = CallFunction<RobotInterface*>(
        "librobotinterface2_" + robot_type + ".so",
        "xbot2_create_robot_plugin_" + robot_type,
        std::move(mdl)
        );

    return UniquePtr(rob);
}

RobotInterface::UniquePtr RobotInterface::getRobot(urdf::ModelConstSharedPtr urdf,
                                              srdf::ModelConstSharedPtr srdf,
                                              std::string robot_type,
                                              std::string model_type)
{
    XBotInterface::ConfigOptions opt { urdf, srdf };

    opt.set_parameter("model_type", model_type);

    opt.set_parameter("robot_type", robot_type);

    auto mdl = ModelInterface::getModel(opt);

    return getRobot(opt);
}

RobotInterface::UniquePtr RobotInterface::getRobot(std::string urdf_string,
                                                  std::string srdf_string,
                                                  std::string robot_type,
                                                  std::string model_type)
{
    XBotInterface::ConfigOptions opt;
    opt.set_urdf(urdf_string);

    if(!srdf_string.empty())
    {
        opt.set_srdf(srdf_string);
    }

    return getRobot(opt.urdf, opt.srdf, robot_type, model_type);
}

RobotInterface::UniquePtr RobotInterface::getRobot(std::string urdf_string,
                                                   std::string robot_type,
                                                   std::string model_type)
{
    XBotInterface::ConfigOptions opt;
    opt.set_urdf(urdf_string);

    return getRobot(opt.urdf, opt.srdf, robot_type, model_type);
}

const ModelInterface &RobotInterface::model() const
{
    return *(r_impl->_model);
}

ModelInterface::ConstPtr RobotInterface::modelSharedPtr() const
{
    return r_impl->_model;
}

RobotJoint::Ptr RobotInterface::getJoint(string_const_ref name)
{
    return getUniversalJoint(name);
}

RobotJoint::Ptr RobotInterface::getJoint(int i)
{
    return getUniversalJoint(i);
}

RobotJoint::ConstPtr RobotInterface::getJoint(string_const_ref name) const
{
    return const_cast<RobotInterface&>(*this).getUniversalJoint(name);
}

RobotJoint::ConstPtr RobotInterface::getJoint(int i) const
{
    return const_cast<RobotInterface&>(*this).getUniversalJoint(i);
}

const std::vector<RobotJoint::Ptr> &RobotInterface::getJoints()
{
    return impl->_joints_rob;
}

const std::vector<RobotJoint::ConstPtr> &RobotInterface::getJoints() const
{
    return impl->_joints_rob_const;
}

bool RobotInterface::setControlMode(const std::map<std::string, ControlMode::Type> &ctrl_map)
{
    bool success = true;

    for(const auto& [jname, jctrl] : ctrl_map)
    {
        if(auto j = getJoint(jname))
        {
            success = j->setControlMode(jctrl) && success;
        }
    }

    return success;
}

bool RobotInterface::setControlMode(const std::map<std::string, ControlMode> &ctrl_map)
{
    bool success = true;

    for(const auto& [jname, jctrl] : ctrl_map)
    {
        if(auto j = getJoint(jname))
        {
            success = j->setControlMode(jctrl) && success;
        }
    }

    return success;
}

void RobotInterface::setReferenceFrom(const XBotInterface &other,
                                      ControlMode::Type mask)
{
    int i = 0;

    if(mask == ControlMode::NONE)
    {
        return;
    }

    for(const auto& jname : getJointNames())
    {
        auto j = getJoint(i);

        i++;

        auto jo = other.getJoint(jname);

        if(!jo)
        {
            continue;
        }

        if(mask & ControlMode::POSITION)
        {
            j->setPositionReference(jo->getJointPosition());
        }

        if(mask & ControlMode::VELOCITY)
        {
            j->setVelocityReference(jo->getJointVelocity());
        }

        if(mask & ControlMode::EFFORT)
        {
            j->setEffortReference(jo->getJointEffort());
        }

    }
}

bool RobotInterface::isUpdated(bool joint_states_only) const
{
    bool is_updated = r_impl->isUpdated();

    if(!is_updated)
    {
        return false;
    }

    if(joint_states_only)
    {
        return is_updated;
    }

    for(auto& [sname, s] : XBotInterface::impl->_sensor_map)
    {
        if(!s->isUpdated())
        {
            return false;
        }
    }

    return true;

}

wall_time RobotInterface::getTimestamp(bool joint_states_only) const
{
    auto ts = r_impl->getTimestamp();

    if(joint_states_only)
    {
        return ts;
    }

    for(auto& [sname, s] : XBotInterface::impl->_sensor_map)
    {
        ts = std::max(ts, s->getTimestamp());
    }

    return ts;
}

RobotInterface::~RobotInterface()
{

}

RobotInterface::RobotInterface(std::unique_ptr<ModelInterface> model):
    XBotInterface(model->impl)  // trick: share state between internal model and robot
{
    r_impl = std::make_unique<Impl>(*this, std::move(model));
}

bool RobotInterface::validateControlMode(string_const_ref jname, ControlMode::Type ctrl)
{
    return true;
}

void RobotInterface::markAsUpdated(wall_time timestamp)
{
    r_impl->markAsUpdated(timestamp);
}


void XBot::RobotInterface::update_impl()
{
    return r_impl->_model->update();
}

int RobotInterface::getLinkId(string_const_ref link_name) const
{
    return r_impl->_model->getLinkId(link_name);
}

void XBot::RobotInterface::getJacobian(int link_id, MatRef J) const
{
    return r_impl->_model->getJacobian(link_id, J);
}

Eigen::Affine3d XBot::RobotInterface::getPose(int link_id) const
{
    return r_impl->_model->getPose(link_id);
}

void XBot::RobotInterface::sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const
{
    return r_impl->_model->sum(q0, v, q1);
}

void XBot::RobotInterface::difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const
{
    return r_impl->_model->difference(q1, q0, v);
}

XBotInterface::JointParametrization XBot::RobotInterface::get_joint_parametrization(string_const_ref jname)
{
    return r_impl->_model->get_joint_parametrization(jname);
}

// impl

RobotInterface::Impl::Impl(RobotInterface &api,
                            std::unique_ptr<ModelInterface> model):
    _api(api),
    _model(std::move(model))
{
    // note: we share all the internal state with the internal model,
    // but we need to set this robot as external api
    // this is needed for validateControlMode to work correctly
    _model->getImpl().setApi(&api);
}

void RobotInterface::Impl::markAsUpdated(wall_time timestamp)
{
    _is_updated = true;
    _js_timestamp = timestamp;
}

bool RobotInterface::Impl::isUpdated() const
{
    return _is_updated;
}

wall_time RobotInterface::Impl::getTimestamp() const
{
    return _js_timestamp;
}

Eigen::Vector6d XBot::RobotInterface::getVelocityTwist(int link_id) const
{
    return r_impl->_model->getVelocityTwist(link_id);
}

Eigen::Vector6d RobotInterface::getAccelerationTwist(int link_id) const
{
    return r_impl->_model->getAccelerationTwist(link_id);
}

Eigen::Vector6d RobotInterface::getJdotTimesV(int link_id) const
{
    return r_impl->_model->getJdotTimesV(link_id);
}

Eigen::Vector3d RobotInterface::getCOM() const
{
    return r_impl->_model->getCOM();
}

void RobotInterface::getCOMJacobian(MatRef J) const
{
    return r_impl->_model->getCOMJacobian(J);
}

Eigen::Vector3d RobotInterface::getCOMJdotTimesV() const
{
    return r_impl->_model->getCOMJdotTimesV();
}

Eigen::Vector3d RobotInterface::getCOMVelocity() const
{
    return r_impl->_model->getCOMVelocity();
}

Eigen::Vector3d RobotInterface::getCOMAcceleration() const
{
    return r_impl->_model->getCOMAcceleration();
}

double RobotInterface::getMass() const
{
    return r_impl->_model->getMass();
}

VecConstRef XBot::RobotInterface::computeInverseDynamics() const
{
    return r_impl->_model->computeInverseDynamics();
}

VecConstRef RobotInterface::computeGravityCompensation() const
{
    return r_impl->_model->computeGravityCompensation();
}

VecConstRef RobotInterface::computeForwardDynamics() const
{
    return r_impl->_model->computeForwardDynamics();
}

MatConstRef RobotInterface::computeInertiaMatrix() const
{
    return r_impl->_model->computeInertiaMatrix();
}

MatConstRef RobotInterface::computeInertiaInverse() const
{
    return r_impl->_model->computeInertiaInverse();
}

VecConstRef RobotInterface::computeNonlinearTerm() const
{
    return r_impl->_model->computeNonlinearTerm();
}

MatConstRef RobotInterface::computeCentroidalMomentumMatrix() const
{
    return r_impl->_model->computeCentroidalMomentumMatrix();
}

Eigen::Vector6d RobotInterface::computeCentroidalMomentum() const
{
    return r_impl->_model->computeCentroidalMomentum();
}
