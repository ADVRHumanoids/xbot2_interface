#ifndef JOINT_H
#define JOINT_H

#include <urdf_model/joint.h>

#include "common/types.h"
#include "common/state_interface.h"
#include "common/visibility.h"

namespace XBot {

inline namespace v2 {

class XBotInterface;

class ModelJoint;

class RobotJoint;


class XBOT2IFC_API Joint : public ReadStateInterface<Joint>
{

public:

    XBOT_DECLARE_SMART_PTR(Joint);

    string_const_ref getName() const;

    string_const_ref getParentLink() const;

    string_const_ref getChildLink() const;

    int getType() const;

    bool isPassive() const;

    urdf::JointConstSharedPtr getUrdfJoint() const;

    const JointInfo& getJointInfo() const;

    int getQIndex() const;

    int getVIndex() const;

    int getId() const;

    bool checkJointLimits(double q, double tolerance = 0) const;

    VecRef getJointPositionMinimal() const;

    void minimalToPosition(VecConstRef q_minimal,
                           VecRef q) const;

    void minimalToPosition(VecConstRef q_minimal,
                           Eigen::VectorXd& q) const;

    void minimalToPosition(double q_minimal,
                           VecRef q) const;

    void minimalToPosition(double q_minimal,
                           Eigen::VectorXd& q) const;

    void positionToMinimal(VecConstRef q,
                           VecRef q_minimal) const;

    void positionToMinimal(VecConstRef q,
                           Eigen::VectorXd& q_minimal) const;

    void positionToMinimal(VecConstRef q,
                           double& q_minimal) const;

    void forwardKinematics(VecConstRef q,
                           VecConstRef v,
                           Eigen::Affine3d& p_T_c,
                           Eigen::Vector6d& c_vc) const;

    void inverseKinematics(const Eigen::Affine3d& p_T_c,
                           const Eigen::Vector6d& c_vc,
                           Eigen::VectorXd& q,
                           Eigen::VectorXd& v) const;

    ~Joint();

    friend class XBotInterface;
    friend ReadStateInterface<Joint>;
    friend ReadCmdInterface<RobotJoint>;

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

    Joint(std::unique_ptr<Impl>);

public:

    Impl& getImpl();

};


class XBOT2IFC_API ModelJoint : public virtual Joint,
        public WriteStateInterface<ModelJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(ModelJoint);

    void setJointPositionMinimal(VecConstRef q);

    void setJointPositionMinimal(double q);

    using WriteStateInterface<ModelJoint>::setJointPosition;

    void setJointPosition(double q);

    using WriteStateInterface<ModelJoint>::setJointVelocity;

    void setJointVelocity(double v);

    using WriteStateInterface<ModelJoint>::setJointEffort;

    void setJointEffort(double tau);

    friend WriteStateInterface<ModelJoint>;

protected:

    using Joint::Joint;

};


class XBOT2IFC_API RobotJoint : public virtual Joint,
        public ReadCmdInterface<RobotJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(RobotJoint);

    void setPositionReferenceMinimal(VecConstRef q);
    VecConstRef getPositionReferenceMinimal() const;
    VecConstRef getPositionReferenceFeedbackMinimal() const;

    friend class RobotInterface;
    friend ReadCmdInterface<RobotJoint>;

protected:

    bool validateControlMode(ControlMode::Type ctrl);

    using Joint::Joint;

};


class XBOT2IFC_API UniversalJoint : public RobotJoint,
                       public ModelJoint,
                       public WriteCmdInterface<UniversalJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(UniversalJoint);

    UniversalJoint(std::unique_ptr<Joint::Impl> impl);

    void setMotorPositionMinimal(double q);

    using WriteCmdInterface<UniversalJoint>::setMotorVelocity;

    void setMotorVelocity(double v);

    using WriteCmdInterface<UniversalJoint>::setPositionReferenceFeedback;

    void setPositionReferenceFeedback(double q);

    void setPositionReferenceFeedbackMinimal(double q);

    using WriteCmdInterface<UniversalJoint>::setVelocityReferenceFeedback;

    void setVelocityReferenceFeedback(double v);  // real posref from robot

    using WriteCmdInterface<UniversalJoint>::setEffortReferenceFeedback;

    void setEffortReferenceFeedback(double tau);  // real posref from robot

    using WriteCmdInterface<UniversalJoint>::setStiffnessFeedback;

    void setStiffnessFeedback(double k);  // real gain from robot

    using WriteCmdInterface<UniversalJoint>::setDampingFeedback;

    void setDampingFeedback(double d);  // real gain from robot

    friend WriteCmdInterface<UniversalJoint>;
};

}

}

#endif // JOINT_H
