#ifndef JOINT_H
#define JOINT_H

#include <urdf_model/joint.h>

#include <xbot2_interface/types.h>
#include <xbot2_interface/state_interface.h>

namespace XBot {

class XBotInterface2;

class Joint : public StateInterface<Joint>
{

public:

    XBOT_DECLARE_SMART_PTR(Joint);

    std::string_view getName() const;

    std::string_view getParentLink() const;

    std::string_view getChildLink() const;

    int getType() const;

    urdf::JointConstSharedPtr getUrdfJoint() const;


    void setJointPosition(double q);

    void setTransform(const Eigen::Affine3d& b_T_c);
    void setVelocity(const Eigen::Vector6d& b_vc);
    void setLocalVelocity(const Eigen::Vector6d& c_vc);

    friend class XBotInterface2;
    friend StateInterface<Joint>;

private:

    class Impl;

    std::unique_ptr<Impl> impl;

    static UniquePtr create(std::unique_ptr<Impl>);

};

}

#endif // JOINT_H