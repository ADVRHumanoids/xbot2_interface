#include "modelinterface2_pin.h"

#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/regressor.hpp>

using namespace XBot;

ModelInterface2Pin::ModelInterface2Pin(const ConfigOptions& opt):
    ModelInterface(opt),
    _cached_computation(None),
    _world_aligned(pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
{
    pinocchio::urdf::buildModel(std::const_pointer_cast<urdf::Model>(getUrdf()), _mdl);
    _data = pinocchio::Data(_mdl);
    _data_no_acc = pinocchio::Data(_mdl);

    _qneutral = pinocchio::neutral(_mdl);

    _tmp.resize(_mdl.nq, _mdl.nv);

    _vzero.setZero(_mdl.nv);

    for(auto& f : _mdl.frames)
    {
        _frame_idx[f.name] = _mdl.getFrameId(f.name);
    }

    finalize();
}

void ModelInterface2Pin::update()
{
    pinocchio::forwardKinematics(_mdl, _data,
                                 getJointPosition(),
                                 getJointVelocity(),
                                 getJointAcceleration());

    pinocchio::updateFramePlacements(_mdl, _data);

    _cached_computation = Kinematics;
}

int XBot::ModelInterface2Pin::getLinkId(string_const_ref link_name) const
{
    try
    {
        return get_frame_id(link_name);
    }
    catch (std::out_of_range&)
    {
        return -1;
    }
}

Eigen::Affine3d ModelInterface2Pin::getPose(int frame_idx) const
{
    Eigen::Affine3d ret;
    ret.translation() = _data.oMf.at(frame_idx).translation();
    ret.linear() = _data.oMf.at(frame_idx).rotation();
    return ret;
}

void ModelInterface2Pin::getJacobian(int link_id, MatRef J) const
{
    if(!(_cached_computation & Jacobians))
    {
        pinocchio::computeJointJacobians(_mdl, _data, getJointPosition());
        _cached_computation |= Jacobians;
    }

    J.setZero();

    pinocchio::getFrameJacobian(_mdl, _data, link_id, _world_aligned, J);
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

MatConstRef ModelInterface2Pin::computeRegressor() const
{
    throw std::runtime_error("not implemented");
}

void ModelInterface2Pin::sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const
{
    pinocchio::integrate(_mdl, q0, v, q1);
}

void ModelInterface2Pin::difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const
{
    v = pinocchio::difference(_mdl, q0, q1);
}

XBotInterface::JointParametrization ModelInterface2Pin::get_joint_parametrization(string_const_ref jname)
{
    JointParametrization ret;

    if(!_mdl.existJointName(std::string(jname)))
    {
        return ret;
    }

    // get id of this joint inside pin model
    size_t pin_id = _mdl.getJointId(std::string(jname));

    // fill required info
    ret.info.id = pin_id;
    ret.info.iq = _mdl.idx_qs[pin_id];
    ret.info.nq = _mdl.nqs[pin_id];
    ret.info.iv = _mdl.idx_vs[pin_id];
    ret.info.nv = _mdl.nvs[pin_id];
    ret.q0 = _qneutral.segment(ret.info.iq, ret.info.nq);

    // urdf joint
    auto jptr = getUrdf()->joints_.at(std::string(jname));

    // tell base class about how to set q from its minimal
    // and maximal representations

    // so(2)
    if(jptr->type == urdf::Joint::CONTINUOUS &&
            ret.info.nq == 2)
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
            ret.info.nq == 7)
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

pinocchio::Index ModelInterface2Pin::get_frame_id(string_const_ref name) const
{
    return _frame_idx.at(name);
}

void ModelInterface2Pin::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    qsum.setZero(nq);
    rnea.setZero(nv);
    qdiff.setZero(nv);
}

Eigen::Vector6d XBot::ModelInterface2Pin::getVelocityTwist(int frame_idx) const
{
    auto v = pinocchio::getFrameVelocity(_mdl, _data, frame_idx, _world_aligned);
    return v;
}

Eigen::Vector6d ModelInterface2Pin::getAccelerationTwist(int frame_idx) const
{
    return pinocchio::getFrameClassicalAcceleration(_mdl, _data, frame_idx, _world_aligned);
}

Eigen::Vector6d ModelInterface2Pin::getJdotTimesV(int frame_idx) const
{
    if(!(_cached_computation & KinematicsNoAcc))
    {
        pinocchio::forwardKinematics(_mdl, _data_no_acc, getJointPosition(), getJointVelocity(), _vzero);
        _cached_computation |= KinematicsNoAcc;
    }

    return pinocchio::getFrameClassicalAcceleration(_mdl, _data_no_acc, frame_idx, _world_aligned);
}

double ModelInterface2Pin::getMass() const
{
    return pinocchio::computeTotalMass(_mdl);
}

Eigen::Vector3d ModelInterface2Pin::getCOM() const
{
    if(!(_cached_computation & Com))
    {
        pinocchio::centerOfMass(_mdl, _data, false);
        _cached_computation |= Com;
    }

    return _data.com[0];
}

void ModelInterface2Pin::getCOMJacobian(MatRef J) const
{
    J = pinocchio::jacobianCenterOfMass(_mdl, _data, false);
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Pin, pin);



