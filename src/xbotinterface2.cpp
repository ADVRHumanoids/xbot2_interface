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
    impl = std::make_shared<Impl>(urdf, srdf, *this);
}

XBotInterface2::XBotInterface2(std::shared_ptr<Impl> _impl):
    impl(_impl)
{

}

ModelInterface2::UniquePtr ModelInterface2::getModel(urdf::ModelConstSharedPtr urdf,
                                                     srdf::ModelConstSharedPtr srdf,
                                                     std::string type)
{
    XBotInterface2::ConfigOptions opt { urdf, srdf };

    auto mdl = CallFunction<ModelInterface2*>("libmodelinterface2_" + type + ".so",
                                              "xbot2_create_model_plugin_" + type,
                                              opt);

    return UniquePtr(mdl);
}

void ModelInterface2::syncFrom(const XBotInterface2 &other)
{
    setJointPosition(other.getJointPosition());
    setJointVelocity(other.getJointVelocity());
    setJointEffort(other.getJointEffort());
    setJointAcceleration(other.getJointAcceleration());
}

urdf::ModelConstSharedPtr XBotInterface2::getUrdf() const
{
    return impl->_urdf;
}

srdf::ModelConstSharedPtr XBotInterface2::getSrdf() const
{
    return impl->_srdf;
}

bool XBotInterface2::hasRobotState(string_const_ref name) const
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

Eigen::VectorXd XBotInterface2::getRobotState(string_const_ref name) const
{
    return impl->getRobotState(name);
}

int XBotInterface2::getJointNum() const
{
    return impl->_joints.size();
}

bool XBotInterface2::hasJoint(string_const_ref name) const
{
    return static_cast<bool>(getJoint(name));
}

ModelJoint::Ptr ModelInterface2::getJoint(string_const_ref name)
{
    return impl->getJoint(name);
}

ModelJoint::Ptr ModelInterface2::getJoint(int i)
{
    return impl->getJoint(i);
}

ModelInterface2::~ModelInterface2()
{

}

Joint::ConstPtr XBotInterface2::getJoint(string_const_ref name) const
{
    return impl->getJoint(name);
}

Joint::ConstPtr XBotInterface2::getJoint(int i) const
{
    return impl->getJoint(i);
}

JointInfo XBotInterface2::getJointInfo(string_const_ref name) const
{
    int jid = getJointId(name);

    if(jid < 0)
    {
        throw std::out_of_range("joint '" + name + "' not found");
    }

    return impl->_joint_info[jid];

}

JointInfo XBotInterface2::getJointInfo(int i) const
{
    if(i >= getJointNum())
    {
        throw std::out_of_range("joint #'" + std::to_string(i) + "' does not exist");
    }

    return impl->_joint_info[i];
}

int XBotInterface2::getJointId(string_const_ref name) const
{
    try
    {
        return impl->_name_id_map.at(name);
    }
    catch(std::out_of_range&)
    {
        return -1;
    }
}

void XBotInterface2::getJacobian(string_const_ref link_name, MatRef J) const
{
    auto res = getJacobian(link_name);
    check_mat_size(J, res, __func__);
    J.noalias() = res;
}

void XBotInterface2::getJacobian(string_const_ref link_name, Eigen::MatrixXd &J) const
{
    J.noalias() = getJacobian(link_name);
}

Eigen::Vector6d XBotInterface2::getVelocityTwist(string_const_ref link_name) const
{
    Eigen::Vector6d ret;
    ret.noalias() = getJacobian(link_name) * getJointVelocity();
    return ret;
}

XBotInterface2::JointParametrization XBotInterface2::get_joint_parametrization(string_const_ref jname)
{
    return impl->get_joint_parametrization(jname);
}

UniversalJoint::Ptr XBotInterface2::getUniversalJoint(string_const_ref name)
{
    return impl->getJoint(name);
}

UniversalJoint::Ptr XBotInterface2::getUniversalJoint(int i)
{
    return impl->getJoint(i);
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

Eigen::VectorXd XBotInterface2::Impl::getRobotState(string_const_ref name) const
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

            if(_joint_info[id].nq > 1 || jval.size() > 1)
            {
                // ignore
                continue;
            }

            qrs[_joint_info[id].iq] = jval[0];
        }

        return qrs;

    }

    throw std::out_of_range("cannot retrieve robot state: no such state");
}

UniversalJoint::Ptr XBotInterface2::Impl::getJoint(string_const_ref name) const
{
    auto it = _name_id_map.find(name);

    if(it == _name_id_map.end())
    {
        return nullptr;
    }

    int i = it->second;
    return _joints.at(i);
}

UniversalJoint::Ptr XBotInterface2::Impl::getJoint(int i) const
{
    return _joints.at(i);
}

XBotInterface2::JointParametrization
XBotInterface2::Impl::get_joint_parametrization(string_const_ref)
{
    JointParametrization ret;

    throw std::runtime_error(std::string(__func__) + " not impl");

    return ret;
}

void XBotInterface2::Impl::finalize()
{
    int nj = 0;
    int nq = 0;
    int nv = 0;

    std::map<std::string, JointParametrization> jparam_map;

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
        if(jparam.info.id < 0)
        {
            throw std::runtime_error("joint " + jname + " not found in implementation");
        }

        // change impl id to our own id
        jparam.info.id = nj;

        // save id, nq, nv, iq, iv, name, whole param
        _joint_name.push_back(jname);

        _joint_info.push_back(jparam.info);

        _name_id_map[jname] = nj;

        jparam_map[jname] = jparam;

        // increment global dimensions
        nq += jparam.info.nq;
        nv += jparam.info.nv;
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
        auto [jid, jiq, jiv, jnq, jnv] = _joint_info[i];

        if(jiq + jnq > nq)
        {
            throw std::runtime_error("joint q index consistency check failed (out of range)");
        }

        if(i < nj - 1 && jiq + jnq != _joint_info[i+1].iq)
        {
            throw std::runtime_error("joint q index consistency check failed (not consecutive)");
        }

        if(jiv + jnv > nv)
        {
            throw std::runtime_error("joint v index consistency check failed (out of range)");
        }

        if(i < nj - 1 && jiv + jnv != _joint_info[i+1].iv)
        {
            throw std::runtime_error("joint v index consistency check failed (not consecutive)");
        }
    }

    // resize state and cmd
    detail::resize(_state, nq, nv, nj);
    detail::resize(_cmd, nq, nv, nj);
    _cmd.ctrlmode.setConstant(~0);
    _qneutral.setZero(nq);

    // set neutral config
    for(int i = 0; i < nj; i++)
    {
        _qneutral.segment(_joint_info[i].iq,
                          _joint_info[i].nq) = jparam_map[_joint_name[i]].q0;
    }

    // use it to initialize cmd and state
    _cmd.qcmd = _state.qref = _state.qmot = _state.qlink = _qneutral;

    // construct internal joints
    for(int i = 0; i < nj; i++)
    {
        auto jname = _joint_name[i];
        auto jptr = _urdf->joints_.at(jname);

        auto [id, iq, iv, nq, nv] = _joint_info[i];

        // create xbot's joint

        // get views on relevant states and control
        auto sv = detail::createView(_state,
                                     iq, nq,
                                     iv, nv);

        auto cv = detail::createView(_cmd,
                                     iq, nq,
                                     iv, nv,
                                     i, 1);

        // create private implementation of joint
        auto jimpl = std::make_unique<Joint::Impl>(sv, cv, jptr);

        // inject mappings
        auto& jparam = jparam_map.at(jname);
        jimpl->fn_minimal_to_q = jparam.fn_minimal_to_q;
        jimpl->fn_q_to_minimal = jparam.fn_q_to_minimal;
        jimpl->fn_fwd_kin = jparam.fn_fwd_kin;
        jimpl->fn_inv_kin = jparam.fn_inv_kin;

        // check we got all info we need
        if(nq != nv &&
                !jimpl->fn_minimal_to_q)
        {
            throw std::runtime_error("fn_minimal_to_q not provided for non-euclidean joint " + jname);
        }

        if(nq != nv &&
                !jimpl->fn_q_to_minimal)
        {
            throw std::runtime_error("fn_q_to_minimal not provided for non-euclidean joint " + jname);
        }

        if(jptr->type == urdf::Joint::FLOATING &&
                nq != nv &&
                !jimpl->fn_fwd_kin)
        {
            throw std::runtime_error("fn_fwd_kin not provided for non-euclidean floating joint " + jname);
        }

        if(jptr->type == urdf::Joint::FLOATING &&
                nq != nv &&
                !jimpl->fn_inv_kin)
        {
            throw std::runtime_error("fn_inv_kin not provided for non-euclidean floating joint " + jname);
        }

        // create and save joint
        auto j =  std::make_shared<UniversalJoint>(std::move(jimpl));
        _joints.push_back(j);

    }

    // trigger model update
    _api.update();

}
