#include <xbot2_interface/common/state_interface.h>
#include "impl/joint.hxx"
#include "impl/xbotinterface2.hxx"
#include "impl/robotinterface2.hxx"
#include "impl/utils.h"

using namespace XBot;

template <typename T1, typename T2, typename T3>
void set_ctrlmask(const T1& ctrlmode, T2 mode, T3& ctrlmask)
{
    for(int i = 0; i < ctrlmask.size(); i++)
    {
        ctrlmask[i] |= (mode & ctrlmode[i]);
    }
}

template <class Derived>
int ReadStateInterface<Derived>::getNq() const
{
    return derived().impl->_state.qlink.size();
}

template <class Derived>
int ReadStateInterface<Derived>::getNv() const
{
    return derived().impl->_state.vlink.size();
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointPosition() const
{
    return derived().impl->_state.qlink;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointPosition(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.qlink, __func__);
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointVelocity() const
{
    return derived().impl->_state.vlink;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointVelocity(VecConstRef v)
{
    check_and_set(v, derived().impl->_state.vlink, __func__);
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointAcceleration() const
{
    return derived().impl->_state.a;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointAcceleration(VecConstRef a)
{
    check_and_set(a, derived().impl->_state.a, __func__);
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointEffort() const
{
    return derived().impl->_state.tau;
}

template<class Derived>
void ReadStateInterface<Derived>::getJointPosition(Eigen::VectorXd &q) const
{
    q = getJointPosition();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointVelocity(Eigen::VectorXd &v) const
{
    v = getJointVelocity();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointAcceleration(Eigen::VectorXd &a) const
{
    a = getJointAcceleration();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointEffort(Eigen::VectorXd &tau) const
{
    tau = getJointEffort();
}

template<class Derived>
std::pair<VecConstRef, VecConstRef> ReadStateInterface<Derived>::getJointLimits() const
{
    return std::make_pair(derived().impl->_state.qmin, derived().impl->_state.qmax);
}

template<class Derived>
void ReadStateInterface<Derived>::getJointLimits(Eigen::VectorXd &qmin, Eigen::VectorXd &qmax) const
{
    std::tie(qmin, qmax) = getJointLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getVelocityLimits() const
{
    return derived().impl->_state.vmax;
}

template<class Derived>
void ReadStateInterface<Derived>::getVelocityLimits(Eigen::VectorXd &vmax) const
{
    vmax = getVelocityLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getEffortLimits() const
{
    return derived().impl->_state.taumax;
}

template<class Derived>
void ReadStateInterface<Derived>::getEffortLimits(Eigen::VectorXd &taumax) const
{
    taumax = getVelocityLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getNeutralQ() const
{
    return derived().impl->_state.qneutral;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointEffort(VecConstRef tau)
{
    check_and_set(tau, derived().impl->_state.tau, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setJointLimits(VecConstRef qmin, VecConstRef qmax)
{
    check_and_set(qmin, derived().impl->_state.qmin, __func__);
    check_and_set(qmax, derived().impl->_state.qmax, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setVelocityLimits(VecConstRef vmax)
{
    check_and_set(vmax, derived().impl->_state.vmax, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setEffortLimits(VecConstRef taumax)
{
    check_and_set(taumax, derived().impl->_state.taumax, __func__);
}

template<class Derived>
void ReadCmdInterface<Derived>::setPositionReference(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.qcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::Position,
                 derived().impl->_cmd.ctrlset);
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getVelocityReference() const
{
    return derived().impl->_cmd.vcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getVelocityReferenceFeedback() const
{
    return derived().impl->_state.vref;
}

template<class Derived>
void ReadCmdInterface<Derived>::setVelocityReference(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.vcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::Velocity,
                 derived().impl->_cmd.ctrlset);
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getStiffnessCmd() const
{
    return derived().impl->_cmd.kcmd;
}

template<class Derived>
CtrlModeVectorConstRef ReadCmdInterface<Derived>::getControlMode() const
{
    return derived().impl->_cmd.ctrlmode;
}

template<class Derived>
void ReadCmdInterface<Derived>::setControlMode(CtrlModeVectorConstRef ctrl)
{
    check_and_set(ctrl, derived().impl->_cmd.ctrlmode, __func__);
}

template<class Derived>
CtrlModeVectorConstRef ReadCmdInterface<Derived>::getValidCommandMask() const
{
    return derived().impl->_cmd.ctrlset;
}

template<class Derived>
void ReadCmdInterface<Derived>::clearCommandMask()
{
    derived().impl->_cmd.ctrlset.setZero();
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getPositionReference() const
{
    return derived().impl->_cmd.qcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getPositionReferenceFeedback() const
{
    return derived().impl->_state.qref;
}

template<class Derived>
Derived& ReadStateInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& ReadStateInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& WriteStateInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& WriteStateInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& ReadCmdInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& ReadCmdInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& WriteCmdInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& WriteCmdInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}


template<class Derived>
void WriteCmdInterface<Derived>::setVelocityReferenceFeedback(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.vref, __func__);
}

template<class Derived>
void WriteCmdInterface<Derived>::setStiffnessFeedback(VecConstRef k)
{
    check_and_set(k, derived().impl->_state.k, __func__);
}

namespace XBot {

template struct ReadStateInterface<Joint>;
template struct WriteStateInterface<ModelJoint>;
template struct ReadCmdInterface<RobotJoint>;
template struct WriteCmdInterface<UniversalJoint>;

template struct ReadStateInterface<XBotInterface>;
template struct WriteStateInterface<ModelInterface>;
template struct ReadCmdInterface<RobotInterface>;
template struct WriteCmdInterface<RobotInterface>;













}
