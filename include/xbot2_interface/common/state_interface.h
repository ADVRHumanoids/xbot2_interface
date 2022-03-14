#ifndef STATE_INTERFACE_H
#define STATE_INTERFACE_H

#include "types.h"

namespace XBot {

template <class Derived>
struct ReadStateInterface
{
    friend Derived;

    int getNq() const;
    int getNv() const;

    VecConstRef getJointPosition() const;
    VecConstRef getJointVelocity() const;
    VecConstRef getJointAcceleration() const;
    VecConstRef getJointEffort() const;

private:

    ReadStateInterface() = default;

    Derived& derived();
    const Derived& derived() const;

};

template <class Derived>
struct WriteStateInterface
{
    friend Derived;

    void setJointPosition(VecConstRef q);

    void setJointVelocity(VecConstRef v);

    void setJointAcceleration(VecConstRef a);

    void setJointEffort(VecConstRef tau);

private:

    WriteStateInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

template <class Derived>
struct ReadCmdInterface
{
    friend Derived;

    VecConstRef getPositionReferenceCmd() const;  // from this interface
    VecConstRef getPositionReference() const;  // real posref from robot

    VecConstRef getStiffnessCmd() const;  // from this interface
    VecConstRef getStiffness() const;  // real gain from robot

private:

    ReadCmdInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

template <class Derived>
struct WriteCmdInterface
{
    friend Derived;

    void setPositionReference(VecConstRef q);  // sets cmd to this interface
    void setPositionReferenceCmd(VecConstRef q);  // real posref from robot

    void setStiffness(VecConstRef q);  // sets cmd to this interface
    void setStiffnessActual(VecConstRef k);  // real gain from robot

private:

    WriteCmdInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

}

#endif // STATE_INTERFACE_H
