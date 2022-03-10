#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace XBot;

ModelInterface2Pin::ModelInterface2Pin(urdf::ModelConstSharedPtr urdf,
                                       srdf::ModelConstSharedPtr srdf):
    XBotInterface2(urdf, srdf)
{
    pinocchio::urdf::buildModel(std::const_pointer_cast<urdf::Model>(urdf), _mdl);
    _data = pinocchio::Data(_mdl);

    _qneutral = pinocchio::neutral(_mdl);

    _tmp.resize(_mdl.nq, _mdl.nv);

}

void ModelInterface2Pin::update()
{
    pinocchio::framesForwardKinematics(_mdl, _data, getJointPosition());
    pinocchio::computeJointJacobians(_mdl, _data, getJointPosition());
}

Eigen::Affine3d ModelInterface2Pin::getPose(std::string_view link_name) const
{
    Eigen::Affine3d ret;
    auto frame_idx = _mdl.getFrameId(std::string(link_name));
    ret.translation() = _data.oMf.at(frame_idx).translation();
    ret.linear() = _data.oMf.at(frame_idx).rotation();
    return ret;
}

MatConstRef ModelInterface2Pin::getJacobian(std::string_view link_name) const
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

XBotInterface2::JointParametrization ModelInterface2Pin::get_joint_parametrization(std::string_view jname)
{
    JointParametrization ret;

    if(!_mdl.existJointName(std::string(jname)))
    {
        return ret;
    }

    size_t pin_id = _mdl.getJointId(std::string(jname));

    ret.id = pin_id;
    ret.iq = _mdl.idx_qs[pin_id];
    ret.nq = _mdl.nqs[pin_id];
    ret.iv = _mdl.idx_vs[pin_id];
    ret.nv = _mdl.nvs[pin_id];
    ret.q0 = _qneutral.segment(ret.iq, ret.nq);

    return ret;


}

void ModelInterface2Pin::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    qsum.setZero(nq);
}
