#include "modelinterface2_pin.h"

#include <xbot2_interface/common/plugin.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace XBot;

ModelInterface2Pin::ModelInterface2Pin(const ConfigOptions& opt):
    ModelInterface2(opt)
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
    pinocchio::computeJointJacobians(_mdl, _data, getJointPosition());
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
    auto frame_idx = _mdl.getFrameId(std::string(link_name));
    pinocchio::getFrameJacobian(_mdl, _data, frame_idx, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, _tmp.J);
    return _tmp.J;
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
    // representation

    // so(2)
    if(jptr->type == urdf::Joint::CONTINUOUS &&
            ret.nq == 2)
    {
        ret.fn_minimal_to_q = [](VecConstRef qminimal, VecRef q)
        {
            q[0] = std::cos(qminimal[0]);
            q[1] = std::sin(qminimal[0]);
        };
    }

    // se(3)
    if(jptr->type == urdf::Joint::FLOATING &&
            ret.nq == 7)
    {
        ret.fn_minimal_to_q = [](VecConstRef qminimal, VecRef q)
        {
            q.head<3>() = qminimal.head<3>();

            Eigen::AngleAxisd rollAngle(qminimal[3], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(qminimal[4], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(qminimal[5], Eigen::Vector3d::UnitZ());

            Eigen::Quaterniond rot_q = yawAngle * pitchAngle * rollAngle;

            q.tail<4>() = rot_q.coeffs();

        };
    }

    // tell base class about how to set q from its maximal
    // representation

    // se(3)
    if(jptr->type == urdf::Joint::FLOATING &&
            ret.nq == 7)
    {
        ret.fn_maximal_to_q = [](const Eigen::Affine3d& T, VecRef q)
        {
            q.head<3>() = T.translation();
            q.tail<4>() = Eigen::Quaterniond(T.linear()).coeffs();
        };
    }

    return ret;


}

void ModelInterface2Pin::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    qsum.setZero(nq);
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Pin, pin);
