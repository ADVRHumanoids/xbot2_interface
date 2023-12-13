#ifndef JOINT_H
#define JOINT_H

#include <urdf_model/joint.h>

#include "common/types.h"
#include "common/state_interface.h"
#include "common/visibility.h"

namespace XBot {

inline namespace v2 {

class XBotInterface2;

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

    urdf::JointConstSharedPtr getUrdfJoint() const;

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

    friend class XBotInterface2;
    friend ReadStateInterface<Joint>;
    friend ReadCmdInterface<RobotJoint>;

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

    Joint(std::unique_ptr<Impl>);

};


class XBOT2IFC_API ModelJoint : public virtual Joint,
        public WriteStateInterface<ModelJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(ModelJoint);

    void setJointPositionMinimal(VecConstRef q);

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

    friend class RobotInterface2;
    friend ReadCmdInterface<RobotJoint>;

protected:

    using Joint::Joint;

};


class XBOT2IFC_API UniversalJoint : public RobotJoint,
                       public ModelJoint,
                       public WriteCmdInterface<UniversalJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(UniversalJoint);

    UniversalJoint(std::unique_ptr<Joint::Impl> impl);

    void setPositionReferenceFeedbackMinimal(VecConstRef q);

    friend WriteCmdInterface<UniversalJoint>;
};

}

}

#endif // JOINT_H
