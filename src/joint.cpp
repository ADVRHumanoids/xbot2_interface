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

const JointInfo& Joint::getJointInfo() const
{
    return impl->_jinfo;
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
    impl->_api = this;
}

void ModelJoint::setJointPositionMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_state.qlink);
}

Joint::Impl::Impl(detail::StateView sv,
                  detail::CommandView cv,
                  urdf::JointConstSharedPtr urdf_joint,
                  JointInfo jinfo):
    _urdf_joint(urdf_joint),
    _state(sv),
    _cmd(cv),
    _jinfo(jinfo)
{
    _q_minimal.setZero(sv.vlink.size());
    _qref_minimal.setZero(sv.vlink.size());
    _qcmd_minimal.setZero(sv.vlink.size());

    // default fw kin for revolute, continuous, prismatic
    fn_fwd_kin = [this](VecConstRef q,
                    VecConstRef v,
                    Eigen::Affine3d* b_T_d,
                    Eigen::Vector6d* b_v_d)
    {
        Eigen::Vector3d axis;
        axis << _urdf_joint->axis.x, _urdf_joint->axis.y, _urdf_joint->axis.z;

        if(b_T_d)
        {
            auto& T = *b_T_d;

            auto pos = _urdf_joint->parent_to_joint_origin_transform.position;
            T.translation() << pos.x, pos.y, pos.z;

            if(_urdf_joint->type == urdf::Joint::PRISMATIC)
            {
                T.translation() += axis*q[0];
            }

            auto rot = _urdf_joint->parent_to_joint_origin_transform.rotation;
            T.linear() = Eigen::Quaterniond(&rot.x).toRotationMatrix();

            if(_urdf_joint->type == urdf::Joint::REVOLUTE ||
                _urdf_joint->type == urdf::Joint::CONTINUOUS)
            {
                Eigen::Matrix<double, 1, 1> qminimal(q[0]);

                if(fn_q_to_minimal)
                {
                    fn_q_to_minimal(q, qminimal);
                }

                T.linear() = T.linear() * Eigen::AngleAxisd(qminimal[0], axis);
            }
        }

        if(b_v_d)
        {
            if(_urdf_joint->type == urdf::Joint::PRISMATIC)
            {
                b_v_d->head<3>() = axis;
                b_v_d->tail<3>().setZero();
            }

            if(_urdf_joint->type == urdf::Joint::REVOLUTE)
            {
                b_v_d->head<3>().setZero();
                b_v_d->tail<3>() = axis;
            }

        }
    };
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
