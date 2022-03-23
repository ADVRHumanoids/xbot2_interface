#include "modelinterface2_pin.h"

#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace XBot;

ModelInterface2Pin::ModelInterface2Pin(const ConfigOptions& opt):
    ModelInterface2(opt),
    _cached_computation(None)
{
    pinocchio::urdf::buildModel(std::const_pointer_cast<urdf::Model>(getUrdf()), _mdl);
    _data = pinocchio::Data(_mdl);

    _qneutral = pinocchio::neutral(_mdl);

    _tmp.resize(_mdl.nq, _mdl.nv);

    finalize();
}

void ModelInterface2Pin::update()
{
    pinocchio::framesForwardKinematics(_mdl, _data, getJointPosition());
    _cached_computation = Kinematics;
}

Eigen::Affine3d ModelInterface2Pin::getPose(string_const_ref link_name) const
{
    Eigen::Affine3d ret;
    auto frame_idx = _mdl.getFrameId(std::string(link_name));
    ret.translation() = _data.oMf.at(frame_idx).translation();
    ret.linear() = _data.oMf.at(frame_idx).rotation();
    return ret;
}

MatConstRef ModelInterface2Pin::getJacobian(string_const_ref link_name) const
{
    if(!(_cached_computation & Jacobians))
    {
        pinocchio::computeJointJacobians(_mdl, _data, getJointPosition());
        _cached_computation |= Jacobians;
    }

    auto frame_idx = _mdl.getFrameId(std::string(link_name));
    pinocchio::getFrameJacobian(_mdl, _data, frame_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _tmp.J);
    return _tmp.J;
}

VecConstRef ModelInterface2Pin::computeInverseDynamics() const
{
    if(!(_cached_computation & Rnea))
    {

        _tmp.rnea = pinocchio::rnea(_mdl, _data,
                                    getJointPosition(),
                                    getJointVelocity(),
                                    getJointAcceleration());

        _cached_computation |= Rnea;

    }

    return _tmp.rnea;
}

VecConstRef ModelInterface2Pin::sum(VecConstRef q0, VecConstRef v) const
{
    pinocchio::integrate(_mdl, q0, v, _tmp.qsum);
    return _tmp.qsum;
}

VecConstRef ModelInterface2Pin::difference(VecConstRef q1, VecConstRef q0) const
{
    return pinocchio::difference(_mdl, q0, q1);
}

XBotInterface2::JointParametrization ModelInterface2Pin::get_joint_parametrization(string_const_ref jname)
{
    JointParametrization ret;

    if(!_mdl.existJointName(std::string(jname)))
    {
        return ret;
    }

    // get id of this joint inside pin model
    size_t pin_id = _mdl.getJointId(std::string(jname));

    // fill required info
    ret.id = pin_id;
    ret.iq = _mdl.idx_qs[pin_id];
    ret.nq = _mdl.nqs[pin_id];
    ret.iv = _mdl.idx_vs[pin_id];
    ret.nv = _mdl.nvs[pin_id];
    ret.q0 = _qneutral.segment(ret.iq, ret.nq);

    // urdf joint
    auto jptr = getUrdf()->joints_.at(std::string(jname));

    // tell base class about how to set q from its minimal
    // and maximal representations

    // so(2)
    if(jptr->type == urdf::Joint::CONTINUOUS &&
            ret.nq == 2)
    {
        ret.fn_minimal_to_q = [](VecConstRef qminimal, VecRef q)
        {
            q[0] = std::cos(qminimal[0]);
            q[1] = std::sin(qminimal[0]);
        };

        ret.fn_q_to_minimal = [](VecConstRef q, VecRef qminimal)
        {
            qminimal[0] = std::atan2(q[1], q[0]);
        };
    }

    // se(3)
    if(jptr->type == urdf::Joint::FLOATING &&
            ret.nq == 7)
    {
        // minimal orientation repr (RPY) to pin's q vector
        ret.fn_minimal_to_q = [](VecConstRef qminimal, VecRef q)
        {
            q.head<3>() = qminimal.head<3>();

            Eigen::AngleAxisd rollAngle(qminimal[3], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(qminimal[4], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(qminimal[5], Eigen::Vector3d::UnitZ());

            // this must represent p_T_c (parent-to-child)
            Eigen::Quaterniond rot_q = rollAngle * pitchAngle * yawAngle;

            q.tail<4>() = rot_q.coeffs();

        };

        // pin's q vector to RPY
        ret.fn_q_to_minimal = [](VecConstRef q, VecRef qminimal)
        {
            qminimal.head<3>() = q.head<3>();
            qminimal.tail<3>() = Eigen::Quaterniond(q.tail<4>()).toRotationMatrix().eulerAngles(0, 1, 2);
        };

        // SE(3) forward kinematics
        ret.fn_fwd_kin = [](VecConstRef q, VecConstRef v,
                Eigen::Affine3d* p_T_c, Eigen::Vector6d* c_vc)
        {
            if(p_T_c)
            {
                p_T_c->translation() = q.head<3>();
                p_T_c->linear() = Eigen::Quaterniond(q.tail<4>()).toRotationMatrix();
            }

            if(c_vc)
            {
                *c_vc = v;
            }
        };

        // SE(3) inverse kinematics
        ret.fn_inv_kin =  [](const Eigen::Affine3d& p_T_c, const Eigen::Vector6d& c_vc,
                VecRef q, VecRef v)
        {
            q.head<3>() = p_T_c.translation();
            q.tail<4>() = Eigen::Quaterniond(p_T_c.linear()).coeffs();
            v = c_vc;
        };

    }

    return ret;

}

void ModelInterface2Pin::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    qsum.setZero(nq);
    rnea.setZero(nv);
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Pin, pin);
