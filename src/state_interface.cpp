#include <xbot2_interface/common/state_interface.h>
#include "impl/joint.hxx"
#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"

using namespace XBot;

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
void WriteStateInterface<Derived>::setJointEffort(VecConstRef tau)
{
    check_and_set(tau, derived().impl->_state.tau, __func__);
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

namespace XBot {

template struct ReadStateInterface<Joint>;
template struct WriteStateInterface<ModelJoint>;
template struct ReadCmdInterface<RobotJoint>;
template struct WriteCmdInterface<RobotJoint>;

template struct ReadStateInterface<XBotInterface2>;
template struct WriteStateInterface<ModelInterface2>;
template struct ReadCmdInterface<RobotInterface2>;
template struct WriteCmdInterface<RobotInterface2>;


}
