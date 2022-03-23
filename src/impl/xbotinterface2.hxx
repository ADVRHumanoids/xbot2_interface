#ifndef XBOTINTERFACE2_HXX
#define XBOTINTERFACE2_HXX

#include <map>
#include <string>
#include <string_view>

#include <xbot2_interface/xbotinterface2.h>
#include "state.hxx"
#include "chain.hxx"
#include "joint.hxx"


namespace XBot {

class XBotInterface2::Impl
{

public:

    friend class XBotInterface2;
    friend class ModelInterface2;
    friend ReadStateInterface<XBotInterface2>;
    friend ReadCmdInterface<RobotInterface2>;
    friend WriteCmdInterface<RobotInterface2>;
    friend WriteStateInterface<ModelInterface2>;

    Impl(urdf::ModelConstSharedPtr urdf,
         srdf::ModelConstSharedPtr srdf,
         XBotInterface2& api);

    Eigen::VectorXd getRobotState(string_const_ref name) const;

    UniversalJoint::Ptr getJoint(string_const_ref name) const;

    UniversalJoint::Ptr getJoint(int i) const;

    JointParametrization get_joint_parametrization(string_const_ref jname);

    void finalize();



private:

    // api
    XBotInterface2& _api;

    // urdf dom
    urdf::ModelConstSharedPtr _urdf;

    // srdf dom
    srdf::ModelConstSharedPtr _srdf;

    // neutral q
    Eigen::VectorXd _qneutral;

    // model state
    detail::State _state;

    // model command
    detail::Command _cmd;

    // joint name -> joint id
    std::map<std::string, int, std::less<>> _name_id_map;

    // joint id -> conf dim, tangent dim
    std::vector<int> _joint_nq, _joint_nv;

    // joint id -> conf index, tangent index
    std::vector<int> _joint_iq, _joint_iv;

    // joint id -> name
    std::vector<std::string> _joint_name;

    // joints
    std::vector<UniversalJoint::Ptr> _joints;

    // chains
    std::map<std::string, Chain::Ptr> _chain_map;




};

}

#endif // XBOTINTERFACE2_HXX
