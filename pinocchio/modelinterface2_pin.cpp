#include "modelinterface2_pin.h"

#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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

    _mdl_orig = _mdl;

    _mdl_zerograv = _mdl;

    _mdl_zerograv.gravity = decltype(_mdl)::Motion::Zero();

    _qneutral = pinocchio::neutral(_mdl);

    _tmp.resize(_mdl.nq, _mdl.nv);

    _eye.setIdentity(_mdl.nv, _mdl.nv);

    _vzero.setZero(_mdl.nv);

    finalize();
}

ModelInterface::UniquePtr ModelInterface2Pin::clone() const
{
    return std::make_unique<ModelInterface2Pin>(getConfigOptions());
}

void ModelInterface2Pin::update_impl()
{
    // handle additional bodies

    // add contributions from all additional bodies
    for(const auto& [unused, ab] : _attached_body_map)
    {
        // restore original inertia
        _mdl.inertias[ab.frame.parent] = _mdl_orig.inertias[ab.frame.parent];

        // update placement
        _mdl.frames[ab.frame_idx].placement = ab.frame.placement;
    }

    for(const auto& [unused, ab] : _attached_body_map)
    {
        // update inertia
        _mdl.inertias[ab.frame.parent] += ab.frame.placement.act(ab.frame.inertia);

    }

    // compute fk
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
    check_frame_idx_throw(frame_idx);

    Eigen::Affine3d ret;
    ret.translation() = _data.oMf.at(frame_idx).translation();
    ret.linear() = _data.oMf.at(frame_idx).rotation();
    return ret;
}

void ModelInterface2Pin::getJacobian(int link_id, MatRef J) const
{
    check_frame_idx_throw(link_id);

    if(!(_cached_computation & Jacobians))
    {
        pinocchio::computeJointJacobians(_mdl, _data, getJointPosition());
        _cached_computation |= Jacobians;
    }

    J.setZero();

    pinocchio::getFrameJacobian(_mdl, _data, link_id, _world_aligned, J);
}


MatConstRef ModelInterface2Pin::computeRegressor() const
{
    throw NotImplemented(__PRETTY_FUNCTION__);
}

void ModelInterface2Pin::sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const
{
    q1.resize(getNq());
    pinocchio::integrate(_mdl, q0, v, q1);
}

void ModelInterface2Pin::difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const
{
    v = pinocchio::difference(_mdl, q0, q1);
}

int ModelInterface2Pin::addFixedLink(string_const_ref link_name,
                                     string_const_ref parent_name,
                                     double mass,
                                     Eigen::Matrix3d inertia,
                                     Eigen::Affine3d p_pose)
{
    // compute pose w.r.t. parent movable joint
    auto parent = getUrdf()->links_.at(parent_name);

    auto movable_joint = parent->parent_joint;

    while(movable_joint->type == urdf::Joint::FIXED)
    {
        movable_joint = getUrdf()->links_.at(movable_joint->parent_link_name)->parent_joint;
    }

    Eigen::Affine3d m_T_p = static_cast<XBotInterface*>(this)->getPose(movable_joint->child_link_name, parent_name);
    auto m_pose = m_T_p * p_pose;

    // exists already
    if(_mdl.existFrame(link_name))
    {
        return get_frame_id(link_name);
    }

    // add frame
    pinocchio::Frame frame;
    frame.parent = _mdl.getJointId(movable_joint->name);
    frame.placement.translation() = m_pose.translation();
    frame.placement.rotation() = m_pose.linear();
    frame.inertia.mass() = mass;
    frame.inertia.inertia() = pinocchio::Symmetric3(inertia);
    frame.name = link_name;
    frame.type = pinocchio::FrameType::BODY;
    frame.previousFrame = -1;

    AttachedBody ab;
    ab.frame = frame;
    ab.frame_idx = _mdl.addBodyFrame(link_name, frame.parent, frame.placement);
    ab.m_T_p = m_T_p;

    _attached_body_map[ab.frame_idx] = ab;

    _data = pinocchio::Data(_mdl);

    return ab.frame_idx;

}

bool ModelInterface2Pin::updateFixedLink(int link_id, double mass, Eigen::Matrix3d inertia, Eigen::Affine3d pose)
{
    auto& ab = _attached_body_map.at(link_id);

    Eigen::Affine3d m_pose = ab.m_T_p * pose;

    ab.frame.inertia.mass() = mass;
    ab.frame.inertia.inertia() = pinocchio::Symmetric3(inertia);
    ab.frame.placement.translation() = m_pose.translation();
    ab.frame.placement.rotation() = m_pose.linear();

    return true;  // TBD check mass and inertia
}

XBotInterface::JointParametrization
ModelInterface2Pin::get_joint_parametrization(string_const_ref jname)
{
    JointParametrization ret;

    if(!_mdl.existJointName(std::string(jname)))
    {
        throw std::out_of_range("joint '" + jname + "' not found in pinocchio model");
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

    auto jorigin = jptr->parent_to_joint_origin_transform;

    Eigen::Affine3d p_T_origin;

    p_T_origin.translation() <<
        jorigin.position.x,
        jorigin.position.y,
        jorigin.position.z;

    Eigen::Quaterniond p_q_origin;

    jorigin.rotation.getQuaternion(
                        p_q_origin.x(),
                        p_q_origin.y(),
                        p_q_origin.z(),
                        p_q_origin.w()
        );

    p_T_origin.linear() = p_q_origin.toRotationMatrix();

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
        ret.fn_fwd_kin = [p_T_origin](VecConstRef q, VecConstRef v,
                Eigen::Affine3d* p_T_c, Eigen::Vector6d* c_vc)
        {
            if(p_T_c)
            {
                p_T_c->translation() = q.head<3>();
                p_T_c->linear() = Eigen::Quaterniond(q.tail<4>()).toRotationMatrix();
                *p_T_c = p_T_origin * *p_T_c;
            }

            if(c_vc)
            {
                *c_vc = v;
            }
        };

        // SE(3) inverse kinematics
        ret.fn_inv_kin =  [p_T_origin](const Eigen::Affine3d& p_T_c, const Eigen::Vector6d& c_vc,
                VecRef q, VecRef v)
        {
            Eigen::Affine3d origin_T_c = p_T_origin.inverse() * p_T_c;
            q.head<3>() = origin_T_c.translation();
            q.tail<4>() = Eigen::Quaterniond(origin_T_c.linear()).coeffs();
            v = c_vc;
        };

    }

    return ret;

}

pinocchio::Index ModelInterface2Pin::get_frame_id(string_const_ref name) const
{
    auto it = _frame_idx.find(name);

    if(it != _frame_idx.end())
    {
        return it->second;
    }

    int idx = _mdl.getFrameId(name);

    check_frame_idx_throw(idx);

    _frame_idx[name] = idx;

    return idx;

}

void ModelInterface2Pin::check_frame_idx_throw(int idx) const
{
    if(idx < 0 || idx >= _mdl.nframes)
    {
        throw std::out_of_range("invalid frame index " + std::to_string(idx));
    }
}

void ModelInterface2Pin::Temporaries::resize(int nq, int nv)
{
    J.setZero(6, nv);
    qsum.setZero(nq);
    h = gcomp = rnea.setZero(nv);
    qdiff.setZero(nv);
}

Eigen::Vector6d XBot::ModelInterface2Pin::getVelocityTwist(int frame_idx) const
{
    check_frame_idx_throw(frame_idx);

    auto v = pinocchio::getFrameVelocity(_mdl, _data, frame_idx, _world_aligned);

    return v;
}

Eigen::Vector6d ModelInterface2Pin::getAccelerationTwist(int frame_idx) const
{
    check_frame_idx_throw(frame_idx);

    return pinocchio::getFrameClassicalAcceleration(_mdl, _data, frame_idx, _world_aligned);
}

Eigen::Vector6d ModelInterface2Pin::getJdotTimesV(int frame_idx) const
{
    check_frame_idx_throw(frame_idx);

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
        pinocchio::centerOfMass(_mdl, _data, pinocchio::KinematicLevel::ACCELERATION, false);
        _cached_computation |= Com;
    }

    return _data.com[0];
}

Eigen::Vector3d ModelInterface2Pin::getCOMVelocity() const
{
    if(!(_cached_computation & Com))
    {
        pinocchio::centerOfMass(_mdl, _data, pinocchio::KinematicLevel::ACCELERATION, false);
        _cached_computation |= Com;
    }

    return _data.vcom[0];
}

Eigen::Vector3d ModelInterface2Pin::getCOMAcceleration() const
{
    if(!(_cached_computation & Com))
    {
        pinocchio::centerOfMass(_mdl, _data, pinocchio::KinematicLevel::ACCELERATION, false);
        _cached_computation |= Com;
    }

    return _data.acom[0];
}

void ModelInterface2Pin::getCOMJacobian(MatRef J) const
{
    J = pinocchio::jacobianCenterOfMass(_mdl, _data, false);
}

Eigen::Vector3d ModelInterface2Pin::getCOMJdotTimesV() const
{
    if(!(_cached_computation & KinematicsNoAcc))
    {
        pinocchio::forwardKinematics(_mdl, _data_no_acc, getJointPosition(), getJointVelocity(), _vzero);
        _cached_computation |= KinematicsNoAcc;
    }

    if(!(_cached_computation & ComNoAcc))
    {
        pinocchio::centerOfMass(_mdl, _data_no_acc, pinocchio::KinematicLevel::ACCELERATION, false);
        _cached_computation |= ComNoAcc;
    }

    return _data.acom[0];
}

XBOT2_REGISTER_MODEL_PLUGIN(ModelInterface2Pin, pin);



