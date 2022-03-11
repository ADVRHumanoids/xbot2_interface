#include <xbot2_interface/common/plugin.h>

#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"
#include "impl/joint.hxx"
#include "impl/load_object.h"

using namespace XBot;

XBotInterface2::XBotInterface2(const XBotInterface2::ConfigOptions &opt):
    XBotInterface2(opt.urdf, opt.srdf)
{

}

XBotInterface2::XBotInterface2(urdf::ModelConstSharedPtr urdf,
                               srdf::ModelConstSharedPtr srdf)
{
    impl = std::make_unique<Impl>(urdf, srdf, *this);
}

XBotInterface2::UniquePtr XBotInterface2::getModel(urdf::ModelConstSharedPtr urdf,
                                                   srdf::ModelConstSharedPtr srdf,
                                                   std::string type)
{
    XBotInterface2::ConfigOptions opt { urdf, srdf };

    auto mdl = CallFunction<XBotInterface2*>("libmodelinterface2_" + type + ".so",
                                             "xbot2_create_model_plugin_" + type,
                                             opt);

    return UniquePtr(mdl);
}

urdf::ModelConstSharedPtr XBotInterface2::getUrdf() const
{
    return impl->_urdf;
}

srdf::ModelConstSharedPtr XBotInterface2::getSrdf() const
{
    return impl->_srdf;
}

bool XBotInterface2::hasRobotState(std::string_view name) const
{
    try
    {
        getRobotState(name);
        return true;
    }
    catch(std::out_of_range& e)
    {
        return false;
    }
}

Eigen::VectorXd XBotInterface2::getRobotState(std::string_view name) const
{
    return impl->getRobotState(name);
}

Joint::Ptr XBotInterface2::getJoint(std::string_view name)
{
    return std::const_pointer_cast<Joint>(const_cast<const XBotInterface2&>(*this).getJoint(name));
}

Joint::Ptr XBotInterface2::getJoint(int i)
{
    return std::const_pointer_cast<Joint>(const_cast<const XBotInterface2&>(*this).getJoint(i));
}

Joint::ConstPtr XBotInterface2::getJoint(std::string_view name) const
{
    auto it = impl->_name_id_map.find(name);

    if(it == impl->_name_id_map.end())
    {
        return nullptr;
    }

    int i = it->second;
    return impl->_joints.at(i);
}

Joint::ConstPtr XBotInterface2::getJoint(int i) const
{
    return impl->_joints.at(i);
}

XBotInterface2::JointParametrization XBotInterface2::get_joint_parametrization(std::string_view jname)
{
    return impl->get_joint_parametrization(jname);
}

void XBotInterface2::finalize()
{
    return impl->finalize();
}

XBotInterface2::~XBotInterface2()
{

}

// impl
XBotInterface2::Impl::Impl(urdf::ModelConstSharedPtr urdf,
                           srdf::ModelConstSharedPtr srdf,
                           XBotInterface2& api):
    _api(api),
    _urdf(urdf),
    _srdf(srdf)
{

}

Eigen::VectorXd XBotInterface2::Impl::getRobotState(std::string_view name) const
{
    if(!_srdf)
    {
        throw std::out_of_range("cannot retrieve robot state: no srdf defined");
    }

    Eigen::VectorXd qrs = _qneutral;

    for(auto& gs: _srdf->getGroupStates())
    {
        if(gs.name_ != name)
        {
            continue;
        }

        for(auto [jname, jval] : gs.joint_values_)
        {
            int id = _name_id_map.at(jname);

            if(_joint_nq[id] > 1 || jval.size() > 1)
            {
                // ignore
                continue;
            }

            qrs[_joint_iq[id]] = jval[0];
        }

        return qrs;

    }

    throw std::out_of_range("cannot retrieve robot state: no such state");
}

XBotInterface2::JointParametrization XBotInterface2::Impl::get_joint_parametrization(std::string_view jname)
{
    JointParametrization ret;

    // tbd

    return ret;
}

void XBotInterface2::Impl::finalize()
{
    int nj = 0;
    int nq = 0;
    int nv = 0;

    std::map<std::string, Eigen::VectorXd> qneutral;

    auto handle_joint = [&](urdf::JointConstSharedPtr jptr)
    {
        auto jtype = jptr->type;

        auto jname = jptr->name;

        if(jtype == urdf::Joint::FIXED)
        {
            return;
        }

        // get parametrization from implementation
        auto jparam = _api.get_joint_parametrization(jname);

        // not found?
        if(jparam.id < 0)
        {
            throw std::runtime_error("joint " + jname + " not found in implementation");
        }

        // save id, nq, nv, iq, iv, name, neutral config
        _joint_name.push_back(jname);

        _joint_nq.push_back(jparam.nq);

        _joint_nv.push_back(jparam.nv);

        _joint_iq.push_back(jparam.iq);

        _joint_iv.push_back(jparam.iv);

        _name_id_map[jname] = nj;

        qneutral[jname] = jparam.q0;

        // increment global dimensions
        nq += jparam.nq;
        nv += jparam.nv;
        nj += 1;
    };

    std::function<void(urdf::LinkConstSharedPtr)> pre_order_traversal =
            [&](urdf::LinkConstSharedPtr root)
    {
        for(auto j : root->child_joints)
        {
            handle_joint(j);
            pre_order_traversal(_urdf->getLink(j->child_link_name));
        }

    };

    // recursivley traverse urdf tree (depth-first)
    pre_order_traversal(_urdf->root_link_);

    // consistency checks
    for(int i = 0; i < nj; i++)
    {
        if(_joint_iq[i] + _joint_nq[i] > nq)
        {
            throw std::runtime_error("joint q index consistency check failed");
        }

        if(_joint_iv[i] + _joint_nv[i] > nv)
        {
            throw std::runtime_error("joint v index consistency check failed");
        }
    }

    // resize state and cmd
    detail::resize(_state, nq, nv);
    detail::resize(_cmd, nq, nv);
    _qneutral.setZero(nq);

    // set neutral config
    for(int i = 0; i < nj; i++)
    {
        _qneutral.segment(_joint_iq[i], _joint_nq[i]) = qneutral[_joint_name[i]];
    }

    // use it to initialize cmd and state
    _cmd.qcmd = _state.qref = _state.qmot = _state.qlink = _qneutral;

    // construct internal joints
    for(int i = 0; i < nj; i++)
    {
        auto jname = _joint_name[i];
        auto jptr = _urdf->joints_.at(jname);

        // create xbot's joint
        auto sv = detail::createView(_state,
                                     _joint_iq[i], _joint_nq[i],
                                     _joint_iv[i], _joint_nv[i]);

        auto cv = detail::createView(_cmd,
                                     _joint_iq[i], _joint_nq[i],
                                     _joint_iv[i], _joint_nv[i]);


        Joint::Ptr j = Joint::create(std::make_unique<Joint::Impl>(sv, cv, jptr));
        _joints.push_back(j);

    }

    // trigger model update
    _api.update();

}
