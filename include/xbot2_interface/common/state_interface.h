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

template <class Derived>
struct CommandInterface
{
    friend Derived;

    VecConstRef getCmdPosition() const;
    VecConstRef getPositionReference() const;
    void setPositionReference(VecConstRef q);

    VecConstRef getStiffness() const;
    void setStiffness(VecConstRef k);
    VecConstRef getCmdStiffness() const;

protected:

    void setCmdPosition(VecConstRef q);
    void setCmdStiffness(VecConstRef k);

private:

    Derived& derived();
    const Derived& derived() const;

};

}

#endif // STATE_INTERFACE_H
