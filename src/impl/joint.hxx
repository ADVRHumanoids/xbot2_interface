#ifndef JOINT_HXX
#define JOINT_HXX

#include <xbot2_interface/types.h>

#include "../joint.h"
#include "state.hxx"



namespace XBot {

class Joint::Impl
{

public:

    friend class Joint;

    XBOT_DECLARE_SMART_PTR(Joint)

    Impl(detail::StateView sv,
         detail::CommandView cv,
         urdf::JointConstSharedPtr urdf_joint);

    friend StateInterface<Joint>;

private:

    urdf::JointConstSharedPtr _urdf_joint;

    detail::StateView _state;

    detail::CommandView _cmd;

    std::function<VecConstRef(const Eigen::Affine3d&)> _set_transform_fn;

    std::function<VecConstRef(const Eigen::Vector6d&)> _set_local_vel_fn;


};

}

#endif // JOINT_HXX