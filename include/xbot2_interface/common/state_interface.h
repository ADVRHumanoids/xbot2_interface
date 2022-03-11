#ifndef STATE_INTERFACE_H
#define STATE_INTERFACE_H

#include "types.h"

namespace XBot {

template <class Derived>
struct StateInterface
{
    friend Derived;

    int getNq() const;
    int getNv() const;

    VecConstRef getJointPosition() const;
    void setJointPosition(VecConstRef q);

    VecConstRef getJointVelocity() const;
    void setJointVelocity(VecConstRef v);

    VecConstRef getJointAcceleration() const;
    void setJointAcceleration(VecConstRef a);

    VecConstRef getJointEffort() const;
    void setJointEffort(VecConstRef tau);



private:

    Derived& derived();
    const Derived& derived() const;

};

}

#endif // STATE_INTERFACE_H
