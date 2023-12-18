#include "impl/xbotinterface2.hxx"
#include "impl/robotinterface2.hxx"

#include "impl/load_object.h"

using namespace XBot;

bool RobotInterface::sense()
{
    for(auto& item : XBotInterface::impl->_sensor_map)
    {
        item.second->clear();
    }

    return sense_impl();
}

bool RobotInterface::move()
{
    return move_impl();
}


RobotInterface::UniquePtr RobotInterface::getRobot(urdf::ModelConstSharedPtr urdf,
                                              srdf::ModelConstSharedPtr srdf,
                                              std::string robot_type,
                                              std::string model_type)
{
    XBotInterface::ConfigOptions opt { urdf, srdf };

    auto mdl = ModelInterface::getModel(urdf, srdf, model_type);

    auto rob = CallFunction<RobotInterface*>(
                "librobotinterface2_" + robot_type + ".so",
                "xbot2_create_robot_plugin_" + robot_type,
                std::move(mdl)
                );

    return UniquePtr(rob);
}

RobotInterface::UniquePtr RobotInterface::getRobot(std::string urdf_string,
                                                  std::string srdf_string,
                                                  std::string robot_type,
                                                  std::string model_type)
{
    XBotInterface::ConfigOptions opt;
    opt.set_urdf(urdf_string);
    opt.set_srdf(srdf_string);

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

RobotJoint::Ptr RobotInterface::getJoint(string_const_ref name)
{
    return getUniversalJoint(name);
}

RobotJoint::Ptr RobotInterface::getJoint(int i)
{
    return getUniversalJoint(i);
}

const std::vector<RobotJoint::Ptr> &RobotInterface::getJoints()
{
    return impl->_joints_rob;
}

const std::vector<RobotJoint::ConstPtr> &RobotInterface::getJoints() const
{
    return impl->_joints_rob_const;
}

void RobotInterface::setControlMode(const std::map<std::string, ControlMode::Type> &ctrl_map)
{
    for(const auto& [jname, jctrl] : ctrl_map)
    {
        if(auto j = getJoint(jname))
        {
            j->setControlMode(jctrl);
        }
    }
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

RobotInterface::~RobotInterface()
{

}

RobotInterface::RobotInterface(std::unique_ptr<XBotInterface> model):
    XBotInterface(model->impl)  // trick: share state between internal model and robot
{
    r_impl = std::make_unique<Impl>(*this, std::move(model));
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
                            std::unique_ptr<XBotInterface> model):
    _api(api),
    _model(std::move(model))
{

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
