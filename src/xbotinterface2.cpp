#include "modelinterface2_pin.h"

#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"
#include "impl/joint.hxx"

using namespace XBot;

XBotInterface2::XBotInterface2(urdf::ModelConstSharedPtr urdf,
                               srdf::ModelConstSharedPtr srdf)
{
    impl = std::make_unique<Impl>(urdf, srdf, *this);
}

XBotInterface2::Ptr XBotInterface2::getModel(urdf::ModelConstSharedPtr urdf,
                                             srdf::ModelConstSharedPtr srdf)
{
    auto ret = std::make_shared<ModelInterface2Pin>(urdf, srdf);
    return ret;
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

VecConstRef XBotInterface2::getJointPosition() const
{
    return impl->getJointPosition();
}

void XBotInterface2::setJointPosition(VecConstRef q)
{
    return impl->setJointPosition(q);
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
    _urdf(urdf),
    _srdf(srdf),
    _api(api)
{

}

VecConstRef XBotInterface2::Impl::getJointPosition() const
{
    return _state.qlink;
}

void XBotInterface2::Impl::setJointPosition(VecConstRef q)
{
    check_and_set(q, _state.qlink, __func__);
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

            if(_id_to_nq[id] > 1 || jval.size() > 1)
            {
                // ignore
                continue;
            }

            qrs[_id_to_iq[id]] = jval[0];
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

    for(auto [jname, jptr] : _urdf->joints_)
    {
        auto jtype = jptr->type;

        if(jtype == urdf::Joint::FIXED)
        {
            continue;
        }

        // get parametrization from implementation
        auto jparam = _api.get_joint_parametrization(jname);

        // joint id
        int id = jparam.id;

        // not found?
        if(id < 0)
        {
            throw std::runtime_error("joint " + jname + " not found");
        }

        // save id, nq, nv, iq, iv, name, neutral config
        _id_to_name.resize(std::max<int>(id+1, _id_to_name.size()));
        _id_to_name[id] = jname;

        _id_to_nq.resize(std::max<int>(id+1, _id_to_nq.size()));
        _id_to_nq[id] = jparam.nq;

        _id_to_nv.resize(std::max<int>(id+1, _id_to_nv.size()));
        _id_to_nv[id] = jparam.nv;

        _id_to_iq.resize(std::max<int>(id+1, _id_to_iq.size()));
        _id_to_iq[id] = jparam.iq;

        _id_to_iv.resize(std::max<int>(id+1, _id_to_iv.size()));
        _id_to_iv[id] = jparam.iv;

        _name_id_map[jname] = id;

        qneutral[jname] = jparam.q0;

        // create xbot's joint
        auto sv = detail::createView(_state, jparam.iq, jparam.nq, jparam.iv, jparam.nv);
        auto cv = detail::createView(_cmd, jparam.iq, jparam.nq, jparam.iv, jparam.nv);
        auto j = std::make_shared<Joint>();
        auto jimp = std::make_unique<Joint::Impl>();

        // increment global dimensions
        nq += jparam.nq;
        nv += jparam.nv;
        nj += 1;

    }

    // resize state and cmd
    detail::resize(_state, nq, nv);
    detail::resize(_cmd, nq, nv);
    _qneutral.setZero(nq);

    // set neutral config
    for(int i = 0; i < nj; i++)
    {
        _qneutral.segment(_id_to_iq[i], _id_to_nq[i]) = qneutral[_id_to_name[i]];
    }

    // use it to initialize cmd and state
    _cmd.qcmd = _state.qref = _state.qmot = _state.qlink = _qneutral;

    // trigger model update
    _api.update();

}
