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

    Impl(urdf::ModelConstSharedPtr urdf,
         srdf::ModelConstSharedPtr srdf,
         XBotInterface2& api);

    Eigen::VectorXd getRobotState(std::string_view name) const;

    int getJointId(std::string_view jname) const;

    int getJointNq(int id) const;
    int getJointNq(std::string_view jname) const;

    int getJointNv(int id) const;
    int getJointNv(std::string_view jname) const;

    JointParametrization get_joint_parametrization(std::string_view jname);

    void finalize();

    friend StateInterface<XBotInterface2>;

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
    std::vector<Joint::Ptr> _joints;

    // chains
    std::map<std::string, Chain::Ptr> _chain_map;




};

}

#endif // XBOTINTERFACE2_HXX
