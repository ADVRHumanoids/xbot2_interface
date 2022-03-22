#include "impl/xbotinterface2.hxx"
#include "impl/robotinterface2.hxx"

#include "impl/load_object.h"

using namespace XBot;

RobotInterface2::UniquePtr RobotInterface2::getRobot(urdf::ModelConstSharedPtr urdf,
                                              srdf::ModelConstSharedPtr srdf,
                                              std::string robot_type,
                                              std::string model_type)
{
    XBotInterface2::ConfigOptions opt { urdf, srdf };

    auto mdl = ModelInterface2::getModel(urdf, srdf, model_type);

    auto rob = CallFunction<RobotInterface2*>(
                "librobotinterface2_" + robot_type + ".so",
                "xbot2_create_robot_plugin_" + robot_type,
                std::move(mdl)
                );

    return UniquePtr(rob);
}

RobotJoint::Ptr RobotInterface2::getJoint(string_const_ref name)
{
    return getUniversalJoint(name);
}

RobotJoint::Ptr RobotInterface2::getJoint(int i)
{
    return getUniversalJoint(i);
}

RobotInterface2::~RobotInterface2()
{

}

RobotInterface2::RobotInterface2(std::unique_ptr<XBotInterface2> model):
    XBotInterface2(model->impl)
{
    r_impl = std::make_unique<Impl>(*this, std::move(model));
    finalize();
}


void XBot::RobotInterface2::update()
{
    return r_impl->_model->update();
}

MatConstRef XBot::RobotInterface2::getJacobian(string_const_ref link_name) const
{
    return r_impl->_model->getJacobian(link_name);
}

Eigen::Affine3d XBot::RobotInterface2::getPose(string_const_ref link_name) const
{
    return r_impl->_model->getPose(link_name);
}

VecConstRef XBot::RobotInterface2::sum(VecConstRef q0, VecConstRef v) const
{
    return r_impl->_model->sum(q0, v);
}

VecConstRef XBot::RobotInterface2::difference(VecConstRef q1, VecConstRef q0) const
{
    return r_impl->_model->difference(q1, q0);
}

XBotInterface2::JointParametrization XBot::RobotInterface2::get_joint_parametrization(string_const_ref jname)
{
    return r_impl->_model->get_joint_parametrization(jname);
}

// impl

RobotInterface2::Impl::Impl(RobotInterface2 &api,
                            std::unique_ptr<XBotInterface2> model):
    _api(api),
    _model(std::move(model))
{

}
