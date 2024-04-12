#ifndef JOINT_HXX
#define JOINT_HXX

#include <xbot2_interface/common/types.h>
#include <xbot2_interface/joint.h>
#include "robotinterface2.hxx"

#include "state.hxx"


namespace XBot {

class Joint::Impl
{

public:

    Impl(detail::StateView sv,
         detail::CommandView cv,
         urdf::JointConstSharedPtr urdf_joint,
         JointInfo jinfo,
         XBotInterface* xbotifc);

    std::function<void(VecConstRef, VecRef)> fn_minimal_to_q;

    std::function<void(VecConstRef, VecRef)> fn_q_to_minimal;

    std::function<void(VecConstRef,
                       VecConstRef,
                       Eigen::Affine3d*,
                       Eigen::Vector6d*)> fn_fwd_kin;

    std::function<void(const Eigen::Affine3d& p_T_c,
                       const Eigen::Vector6d& c_vc,
                       VecRef q,
                       VecRef v)> fn_inv_kin;

    void setApi(XBotInterface* api);

    friend Joint;

    friend ModelJoint;

    friend RobotJoint;

    friend UniversalJoint;

    friend ReadStateInterface<Joint>;

    friend WriteStateInterface<ModelJoint>;

    friend ReadCmdInterface<RobotJoint>;

    friend WriteCmdInterface<UniversalJoint>;

private:

    Joint * _api;

    XBotInterface * _xbotifc;

    ModelInterface * _modelifc;

    RobotInterface * _robotifc;

    urdf::JointConstSharedPtr _urdf_joint;

    JointInfo _jinfo;

    detail::StateView _state;

    detail::CommandView _cmd;

    Eigen::VectorXd _q_minimal, _qref_minimal, _qcmd_minimal;

    struct Temporaries
    {
        Eigen::VectorXd q;

        void resize(int nq, int nv);
    };

    Temporaries _tmp;

};

}

#endif // JOINT_HXX
