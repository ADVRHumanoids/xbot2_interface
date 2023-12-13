#ifndef STATE_INTERFACE_H
#define STATE_INTERFACE_H

#include "types.h"

namespace XBot {

inline namespace v2 {

template <class Derived>
struct XBOT2IFC_API ReadStateInterface
{
    friend Derived;

    int getNq() const;
    int getNv() const;

    VecConstRef getJointPosition() const;
    VecConstRef getJointVelocity() const;
    VecConstRef getJointAcceleration() const;
    VecConstRef getJointEffort() const;

    void getJointPosition(Eigen::VectorXd& q) const;
    void getJointVelocity(Eigen::VectorXd& v) const;
    void getJointAcceleration(Eigen::VectorXd& a) const;
    void getJointEffort(Eigen::VectorXd& tau) const;

private:

    ReadStateInterface() = default;

    Derived& derived();
    const Derived& derived() const;

};

template <class Derived>
struct XBOT2IFC_API WriteStateInterface
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
struct XBOT2IFC_API ReadCmdInterface
{
    friend Derived;

    VecConstRef getPositionReference() const;  // from this interface
    VecConstRef getPositionReferenceFeedback() const;  // real posref from robot
    void setPositionReference(VecConstRef q);  // sets cmd to this interface

    VecConstRef getVelocityReference() const;  // from this interface
    VecConstRef getVelocityReferenceFeedback() const;  // real posref from robot
    void setVelocityReference(VecConstRef q);  // sets cmd to this interface

    VecConstRef getStiffnessCmd() const;  // from this interface
    VecConstRef getStiffnessFeedback() const;  // real gain from robot
    void setStiffness(VecConstRef q);  // sets cmd to this interface

    CtrlModeVectorConstRef getControlMode() const;
    void setControlMode(CtrlModeVectorConstRef ctrl);
    CtrlModeVectorConstRef getValidCommandMask() const;
    void clearCommandMask();


private:

    ReadCmdInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

template <class Derived>
struct XBOT2IFC_API WriteCmdInterface
{
    friend Derived;

    void setPositionReferenceFeedback(VecConstRef q);  // real posref from robot
    void setVelocityReferenceFeedback(VecConstRef q);  // real posref from robot

    void setStiffnessFeedback(VecConstRef k);  // real gain from robot

private:

    WriteCmdInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

}

}

#endif // STATE_INTERFACE_H
