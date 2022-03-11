#include "impl/joint.hxx"
#include <iostream>

using namespace XBot;

std::string_view Joint::getName() const
{
    return getUrdfJoint()->name;
}

std::string_view Joint::getParentLink() const
{
    return getUrdfJoint()->parent_link_name;
}

std::string_view Joint::getChildLink() const
{
    return getUrdfJoint()->child_link_name;
}

int Joint::getType() const
{
    return getUrdfJoint()->type;
}

urdf::JointConstSharedPtr Joint::getUrdfJoint() const
{
    return impl->_urdf_joint;
}

void Joint::setJointPosition(double q)
{
    impl->_state.qlink[0] = q;
}

void Joint::setJointVelocity(double v)
{
    impl->_state.vlink[0] = v;
}

void Joint::setJointEffort(double tau)
{
    impl->_state.tau[0] = tau;
}

Joint::UniquePtr Joint::create(std::unique_ptr<Impl> impl)
{
    auto ret = std::make_unique<Joint>();
    ret->impl = std::move(impl);
    return ret;
}

Joint::Impl::Impl(detail::StateView sv,
                  detail::CommandView cv,
                  urdf::JointConstSharedPtr urdf_joint):
    _urdf_joint(urdf_joint),
    _state(sv),
    _cmd(cv)
{
    std::cout << "joint " << urdf_joint->name << "\n"
         << sv.qlink.transpose() << "\n"
         << sv.vlink.transpose() << "\n";
}
