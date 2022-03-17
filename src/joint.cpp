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

void Joint::setJointPositionMinimal(VecConstRef q)
{
    if(impl->fn_minimal_to_q)
    {
        impl->fn_minimal_to_q(q, impl->_state.qlink);
        return;
    }

    StateInterface<Joint>::setJointPosition(q);

}

void Joint::setJointPositionMinimal(double q)
{
    Eigen::Matrix<double, 1, 1> _q(q);
    setJointPositionMinimal(_q);
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
