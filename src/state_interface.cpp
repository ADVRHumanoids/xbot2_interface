#include <xbot2_interface/common/state_interface.h>
#include "impl/joint.hxx"
#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"

using namespace XBot;

template <class Derived>
int StateInterface<Derived>::getNq() const
{
    return derived().impl->_state.qlink.size();
}

template <class Derived>
int StateInterface<Derived>::getNv() const
{
    return derived().impl->_state.vlink.size();
}

template <class Derived>
VecConstRef StateInterface<Derived>::getJointPosition() const
{
    return derived().impl->_state.qlink;
}

template<class Derived>
void StateInterface<Derived>::setJointPosition(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.qlink, __func__);
}

template <class Derived>
VecConstRef StateInterface<Derived>::getJointVelocity() const
{
    return derived().impl->_state.vlink;
}

template<class Derived>
void StateInterface<Derived>::setJointVelocity(VecConstRef v)
{
    check_and_set(v, derived().impl->_state.vlink, __func__);
}

template <class Derived>
VecConstRef StateInterface<Derived>::getJointAcceleration() const
{
    return derived().impl->_state.a;
}

template<class Derived>
void StateInterface<Derived>::setJointAcceleration(VecConstRef a)
{
    check_and_set(a, derived().impl->_state.a, __func__);
}

template <class Derived>
VecConstRef StateInterface<Derived>::getJointEffort() const
{
    return derived().impl->_state.tau;
}

template<class Derived>
void StateInterface<Derived>::setJointEffort(VecConstRef tau)
{
    check_and_set(tau, derived().impl->_state.tau, __func__);
}

template<class Derived>
Derived& StateInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& StateInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

namespace XBot {
template struct StateInterface<Joint>;
template struct StateInterface<XBotInterface2>;
}
