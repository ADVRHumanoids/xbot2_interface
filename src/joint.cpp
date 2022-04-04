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

VecRef Joint::getJointPositionMinimal() const
{
    positionToMinimal(getJointPosition(), impl->_q_minimal);
    return impl->_q_minimal;
}

void Joint::minimalToPosition(VecConstRef q_minimal, VecRef q) const
{
    // output size check
    check_mat_size(q_minimal, getNv(), 1, __func__);
    check_mat_size(q, getNq(), 1, __func__);

    // if any mapping needs to be done, invoke handler
    if(impl->fn_minimal_to_q)
    {
        impl->fn_minimal_to_q(q_minimal, q);
        return;
    }

    // otherwise just fill the buffer
    check_and_set(q_minimal, q, __func__);
}

void Joint::minimalToPosition(VecConstRef q_minimal, Eigen::VectorXd &q) const
{
    q.resize(getNq());
    return minimalToPosition(q_minimal, VecRef(q));
}

void Joint::minimalToPosition(double q_minimal, VecRef q) const
{
    Eigen::Scalard _q_minimal(q_minimal);
    minimalToPosition(_q_minimal, q);
}

void Joint::minimalToPosition(double q_minimal, Eigen::VectorXd &q) const
{
    q.resize(getNq());
    return minimalToPosition(q_minimal, VecRef(q));
}

void Joint::positionToMinimal(VecConstRef q, VecRef q_minimal) const
{
    // output size check
    check_mat_size(q_minimal, getNv(), 1, __func__);
    check_mat_size(q, getNq(), 1, __func__);

    // if any mapping needs to be done, invoke handler
    if(impl->fn_q_to_minimal)
    {
        impl->fn_q_to_minimal(q, q_minimal);
        return;
    }

    // otherwise just fill the buffer
    check_and_set(q, q_minimal, __func__);
}

void Joint::positionToMinimal(VecConstRef q, Eigen::VectorXd &q_minimal) const
{
    q_minimal.resize(getNv());
    positionToMinimal(q, VecRef(q_minimal));
}

void Joint::positionToMinimal(VecConstRef q, double &q_minimal) const
{
    Eigen::Scalard ret;
    positionToMinimal(q, ret);
    q_minimal = ret.value();
}

void Joint::forwardKinematics(VecConstRef q,
                              VecConstRef v,
                              Eigen::Affine3d& p_T_c,
                              Eigen::Vector6d& c_vc) const
{
    if(getType() != urdf::Joint::FLOATING)
    {
        throw std::runtime_error("forwardKinematics only implemented for floating joints");
    }

    impl->fn_fwd_kin(q, v, &p_T_c, &c_vc);
}

void Joint::inverseKinematics(const Eigen::Affine3d &p_T_c,
                              const Eigen::Vector6d &c_vc,
                              Eigen::VectorXd &q,
                              Eigen::VectorXd &v) const
{
    if(getType() != urdf::Joint::FLOATING)
    {
        throw std::runtime_error("inverseKinematics only implemented for floating joints");
    }

    q.resize(getNq());
    v.resize(getNv());

    impl->fn_inv_kin(p_T_c, c_vc, q, v);

}

Joint::Joint(std::unique_ptr<Joint::Impl> _impl):
    impl(std::move(_impl))
{
    impl->api = this;
}

void ModelJoint::setJointPositionMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_state.qlink);
}

Joint::Impl::Impl(detail::StateView sv,
                  detail::CommandView cv,
                  urdf::JointConstSharedPtr urdf_joint):
    _urdf_joint(urdf_joint),
    _state(sv),
    _cmd(cv)
{
    _q_minimal.setZero(sv.vlink.size());
    _qref_minimal.setZero(sv.vlink.size());
    _qcmd_minimal.setZero(sv.vlink.size());
}

UniversalJoint::UniversalJoint(std::unique_ptr<Joint::Impl> impl):
    Joint(std::move(impl)),
    RobotJoint(nullptr),
    ModelJoint(nullptr)
{

}

void UniversalJoint::setPositionReferenceFeedbackMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_state.qref);
}

void RobotJoint::setPositionReferenceMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_cmd.qcmd);
    setPositionReference(impl->_cmd.qcmd);
}

VecConstRef RobotJoint::getPositionReferenceMinimal() const
{
    positionToMinimal(getPositionReference(), impl->_qcmd_minimal);
    return impl->_qcmd_minimal;
}

VecConstRef RobotJoint::getPositionReferenceFeedbackMinimal() const
{
    positionToMinimal(getPositionReferenceFeedback(), impl->_qref_minimal);
    return impl->_qref_minimal;
}
