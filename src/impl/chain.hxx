#ifndef CHAIN_HXX
#define CHAIN_HXX

#include <list>
#include <Eigen/Dense>

#include <xbot2_interface/chain.h>
#include "state.hxx"
#include "xbotinterface2.hxx"

namespace XBot {

class Chain::Impl
{

public:

    Impl(detail::StateView sv,
         detail::CommandView cv,
         std::string name,
         std::string base_link,
         std::string tip_link,
         std::list<UniversalJoint::Ptr> joints);

    friend class Chain;
    friend ReadStateInterface<Chain>;

private:

    detail::StateView _state;

    detail::CommandView _cmd;

    std::string _name;

    std::string _base_link;

    std::string _tip_link;

    // joints
    std::vector<std::string> _joint_names;
    std::vector<UniversalJoint::Ptr> _joints;
    std::vector<Joint::Ptr> _joints_xbi;
    std::vector<Joint::ConstPtr> _joints_xbi_const;
    std::vector<ModelJoint::Ptr> _joints_mdl;
    std::vector<ModelJoint::ConstPtr> _joints_mdl_const;
    std::vector<RobotJoint::Ptr> _joints_rob;
    std::vector<RobotJoint::ConstPtr> _joints_rob_const;


};

}

#endif // CHAIN_HXX
