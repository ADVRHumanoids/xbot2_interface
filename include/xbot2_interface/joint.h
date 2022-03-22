#ifndef JOINT_H
#define JOINT_H

#include <urdf_model/joint.h>

#include "common/types.h"
#include "common/state_interface.h"

namespace XBot {

class XBotInterface2;

class ModelJoint;

class RobotJoint;

class Joint : public ReadStateInterface<Joint>
{

public:

    XBOT_DECLARE_SMART_PTR(Joint);

    string_const_ref getName() const;

    string_const_ref getParentLink() const;

    string_const_ref getChildLink() const;

    int getType() const;

    urdf::JointConstSharedPtr getUrdfJoint() const;

    void minimalToPosition(VecConstRef q_minimal,
                           VecRef q) const;

    void minimalToPosition(double q_minimal,
                           VecRef q) const;

    void maximalToPosition(const Eigen::Affine3d& p_T_c,
                           VecRef q) const;

    friend class XBotInterface2;
    friend ReadStateInterface<Joint>;
    friend ReadCmdInterface<RobotJoint>;

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

    Joint(std::unique_ptr<Impl>);

};

class ModelJoint : public virtual Joint,
        public WriteStateInterface<ModelJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(ModelJoint);

    void setJointPositionMinimal(VecConstRef q);
    void setJointPositionMinimal(double q);
    void setJointPositionMaximal(const Eigen::Affine3d& b_T_c);
//    void setVelocity(const Eigen::Vector6d& b_vc);
//    void setLocalVelocity(const Eigen::Vector6d& c_vc);

    friend WriteStateInterface<ModelJoint>;

protected:

    using Joint::Joint;

};

class RobotJoint : public virtual Joint,
        public ReadCmdInterface<RobotJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(RobotJoint);

    friend class RobotInterface2;

    using ReadCmdInterface<RobotJoint>::setPositionReference;
    void setPositionReference(double qref);

protected:

    using Joint::Joint;

};

class UniversalJoint : public RobotJoint,
                       public ModelJoint,
                       public WriteCmdInterface<UniversalJoint>
{

public:

    XBOT_DECLARE_SMART_PTR(UniversalJoint);

    UniversalJoint(std::unique_ptr<Joint::Impl> impl);
};

}

#endif // JOINT_H
