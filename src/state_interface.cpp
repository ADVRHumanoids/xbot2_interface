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
void WriteStateInterface<Derived>::setJointEffort(VecConstRef tau)
{
    check_and_set(tau, derived().impl->_state.tau, __func__);
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
