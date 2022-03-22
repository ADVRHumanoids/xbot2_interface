#include "impl/joint.hxx"
#include "impl/utils.h"

#include <iostream>

using namespace XBot;

string_const_ref Joint::getName() const
{
    return getUrdfJoint()->name;
}

string_const_ref Joint::getParentLink() const
{
    return getUrdfJoint()->parent_link_name;
}

string_const_ref Joint::getChildLink() const
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

void Joint::minimalToPosition(VecConstRef q_minimal, VecRef q) const
{


    // if any mapping needs to be done, invoke handler
    if(impl->fn_minimal_to_q)
    {
        check_vec_size(q_minimal, getNv(), __func__);
        impl->fn_minimal_to_q(q_minimal, q);
        return;
    }

    // otherwise just fill the buffer
    check_mat_size(q_minimal, q, __func__);
    q = q_minimal;
}

void Joint::minimalToPosition(double q_minimal, VecRef q) const
{
    Eigen::Matrix<double, 1, 1> _q_minimal(q_minimal);
    minimalToPosition(_q_minimal, q);
}

Joint::Joint(std::unique_ptr<Joint::Impl> _impl):
    impl(std::move(_impl))
{

}

void ModelJoint::setJointPositionMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_state.qlink);
}

void ModelJoint::setJointPositionMinimal(double q)
{
    Eigen::Matrix<double, 1, 1> _q(q);
    setJointPositionMinimal(_q);
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

UniversalJoint::UniversalJoint(std::unique_ptr<Joint::Impl> impl):
    Joint(std::move(impl)),
    RobotJoint(nullptr),
    ModelJoint(nullptr)
{

}

void RobotJoint::setPositionReference(double qref)
{
    Eigen::Matrix<double, 1, 1> _qref(qref);
    setPositionReference(_qref);
}
