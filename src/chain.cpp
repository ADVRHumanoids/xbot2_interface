#include "impl/chain.hxx"

using namespace XBot;

Chain::Impl::Impl(detail::StateView sv,
         detail::CommandView cv,
         std::string name,
         std::string base_link,
         std::string tip_link,
         std::list<UniversalJoint::Ptr> chain_joints):
    _state(sv),
    _cmd(cv),
    _name(name),
    _base_link(base_link),
    _tip_link(tip_link)
{
    // add joints to all members
    _joints.assign(chain_joints.begin(), chain_joints.end());
    _joints_xbi.assign(chain_joints.begin(), chain_joints.end());
    _joints_mdl.assign(chain_joints.begin(), chain_joints.end());
    _joints_rob.assign(chain_joints.begin(), chain_joints.end());
    _joints_xbi_const.assign(chain_joints.begin(), chain_joints.end());
    _joints_mdl_const.assign(chain_joints.begin(), chain_joints.end());
    _joints_rob_const.assign(chain_joints.begin(), chain_joints.end());

    // add joint names
    for(auto j : _joints)
    {
        _joint_names.push_back(j->getName());
    }

}

string_const_ref Chain::getName() const
{
    return impl->_name;
}

string_const_ref Chain::getBaseLink() const
{
    return impl->_base_link;
}

string_const_ref Chain::getTipLink() const
{
    return impl->_tip_link;
}

int Chain::getJointNum() const
{
    return impl->_joints.size();
}

int Chain::getQIndex() const
{
    return impl->_joints[0]->getQIndex();
}

int Chain::getVIndex() const
{
    return impl->_joints[0]->getVIndex();
}

const std::vector<std::string> &Chain::getJointNames() const
{
    return impl->_joint_names;
}

const std::vector<Joint::Ptr> &Chain::getJoints()
{
    return impl->_joints_xbi;
}

const std::vector<Joint::ConstPtr> &Chain::getJoints() const
{
    return impl->_joints_xbi_const;
}

Chain::Chain(std::unique_ptr<Impl> impl):
    impl(std::move(impl))
{

}

Chain::~Chain()
{

}
