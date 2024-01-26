#ifndef XBOT2IFC_STATE_INTERFACE_H
#define XBOT2IFC_STATE_INTERFACE_H

#include "types.h"

namespace XBot {

inline namespace v2 {

template <class Derived>
struct XBOT2IFC_API ReadStateInterface
{

    int getNq() const;

    int getNv() const;

    const std::vector<std::string>& getQNames() const;

    const std::vector<std::string>& getVNames() const;

    VecConstRef getJointPosition() const;

    VecConstRef getJointVelocity() const;

    VecConstRef getJointAcceleration() const;

    VecConstRef getJointEffort() const;

    /**
     * @brief returns the robot joint limits (initialized from URDF)
     * @return (qmin, qmax)
     * @note joint limits are velocity vectors (size = nv),
     *  and they are intended as
     * the min/max velocity that can be applied for unit time, starting
     * from neutral q
     */
    std::pair<VecConstRef, VecConstRef> getJointLimits() const;

    VecConstRef getVelocityLimits() const;

    VecConstRef getEffortLimits() const;

    /**
     * @brief returns the robot's neutral joint configuration
     * (i.e., its zero configuration, taking into account parametrization of
     * non-euclidean joints)
     * @return q
     */
    VecConstRef getNeutralQ() const;

    void getJointPosition(Eigen::VectorXd& q) const;

    void getJointVelocity(Eigen::VectorXd& v) const;

    void getJointAcceleration(Eigen::VectorXd& a) const;

    void getJointEffort(Eigen::VectorXd& tau) const;

    void getJointLimits(Eigen::VectorXd& qmin, Eigen::VectorXd& qmax) const;

    void getVelocityLimits(Eigen::VectorXd& vmax) const;

    void getEffortLimits(Eigen::VectorXd& taumax) const;

    // map based accessors

    void getJointPosition(JointNameMap& q) const;

    void getJointVelocity(JointNameMap& v) const;

    void getJointAcceleration(JointNameMap& a) const;

    void getJointEffort(JointNameMap& tau) const;

    friend Derived;

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

    void setJointLimits(VecConstRef qmin, VecConstRef qmax);

    void setVelocityLimits(VecConstRef vmax);

    void setEffortLimits(VecConstRef taumax);

    void setJointPosition(const JointNameMap& q);

    void setJointVelocity(const JointNameMap& v);

    void setJointAcceleration(const JointNameMap& a);

    void setJointEffort(const JointNameMap& tau);

private:

    WriteStateInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

template <class Derived>
struct XBOT2IFC_API ReadCmdInterface
{
    friend Derived;

    VecConstRef getMotorPosition() const;
    void getMotorPosition(Eigen::VectorXd& q) const;

    VecConstRef getPositionReference() const;  // from this interface
    void getPositionReference(Eigen::VectorXd& q) const;

    VecConstRef getPositionReferenceFeedback() const;  // real posref from robot
    void getPositionReferenceFeedback(Eigen::VectorXd& q) const;

    void setPositionReference(VecConstRef q);  // sets cmd to this interface

    VecConstRef getMotorVelocity() const;
    void getMotorVelocity(Eigen::VectorXd& q) const;

    VecConstRef getVelocityReference() const;  // from this interface
    void getVelocityReference(Eigen::VectorXd& q) const;

    VecConstRef getVelocityReferenceFeedback() const;  // real posref from robot
    void getVelocityReferenceFeedback(Eigen::VectorXd& q) const;

    void setVelocityReference(VecConstRef q);  // sets cmd to this interface

    VecConstRef getEffortReference() const;  // from this interface
    void getEffortReference(Eigen::VectorXd& q) const;

    VecConstRef getEffortReferenceFeedback() const;  // real posref from robot
    void getEffortReferenceFeedback(Eigen::VectorXd& q) const;

    void setEffortReference(VecConstRef q);  // sets cmd to this interface

    VecConstRef getStiffnessDesired() const;  // from this interface
    void getStiffnessDesired(Eigen::VectorXd& q) const;

    VecConstRef getStiffness() const;  // real gain from robot
    void getStiffness(Eigen::VectorXd& q) const;

    void setStiffness(VecConstRef q);  // sets cmd to this interface

    VecConstRef getDampingDesired() const;  // from this interface
    void getDampingDesired(Eigen::VectorXd& q) const;

    VecConstRef getDamping() const;  // real gain from robot
    void getDamping(Eigen::VectorXd& q) const;

    void setDamping(VecConstRef q);  // sets cmd to this interface

    CtrlModeVectorConstRef getControlMode() const;
    void getControlMode(CtrlModeMap& ctrl) const;
    void getControlMode(CtrlModeTypeMap& ctrl) const;

    bool setControlMode(CtrlModeVectorConstRef ctrl);
    bool setControlMode(ControlMode::Type ctrl);
    bool setControlMode(CtrlModeMap ctrl);
    bool setControlMode(CtrlModeTypeMap ctrl);

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
    void setMotorPosition(VecConstRef q);
    void setMotorVelocity(VecConstRef v);
    void setPositionReferenceFeedback(VecConstRef q);  // real posref from robot
    void setVelocityReferenceFeedback(VecConstRef v);  // real posref from robot
    void setEffortReferenceFeedback(VecConstRef tau);  // real posref from robot
    void setStiffnessFeedback(VecConstRef k);  // real gain from robot
    void setDampingFeedback(VecConstRef d);  // real gain from robot

private:

    WriteCmdInterface() = default;

    Derived& derived();
    const Derived& derived() const;
};

}

}

#endif // STATE_INTERFACE_H
