#include <xbot2_interface/common/state_interface.h>
#include "impl/joint.hxx"
#include "impl/xbotinterface2.hxx"
#include "impl/robotinterface2.hxx"
#include "impl/utils.h"

#define GENERATE_GETTER_IMPL(IfcName, Name, Var) \
template <class Derived> \
VecConstRef IfcName<Derived>::get##Name() const \
{ \
    return derived().impl->Var; \
}

#define GENERATE_SETTER_IMPL(IfcName, Name, Var) \
template<class Derived> \
void IfcName<Derived>::set##Name(VecConstRef q) \
{ \
    check_and_set(q, derived().impl->Var, __func__); \
}

#define GENERATE_MAP_GETTER_IMPL(IfcName, Name, Var, q_or_v) \
template<class Derived> \
void IfcName<Derived>::get##Name(JointNameMap &qmap) const \
{ \
    detail::q_or_v##ToMap(derived().impl->_state, \
                   derived().impl->Var, \
                   qmap); \
}

#define GENERATE_MAP_SETTER_IMPL(IfcName, Name, Var, Q_or_V) \
template<class Derived> \
void IfcName<Derived>::set##Name(const JointNameMap &q) \
{ \
    detail::mapTo##Q_or_V(derived().impl->_state, q, derived().impl->Var); \
}


using namespace XBot;

template <typename T1, typename T2, typename T3>
void set_ctrlmask(const T1& ctrlmode, T2 mode, T3& ctrlmask)
{
    for(int i = 0; i < ctrlmask.size(); i++)
    {
        ctrlmask[i] |= (mode & ctrlmode[i]);
    }
}

template <class Derived>
int ReadStateInterface<Derived>::getNq() const
{
    return derived().impl->_state.qlink.size();
}

template <class Derived>
int ReadStateInterface<Derived>::getNv() const
{
    return derived().impl->_state.vlink.size();
}

template<class Derived>
const std::vector<std::string> &ReadStateInterface<Derived>::getQNames() const
{
    return derived().impl->_state.qnames;
}

template<class Derived>
const std::vector<std::string> &ReadStateInterface<Derived>::getVNames() const
{
    return derived().impl->_state.vnames;
}

// template <class Derived>
// VecConstRef ReadStateInterface<Derived>::getJointPosition() const
// {
//     return derived().impl->_state.qlink;
// }

GENERATE_GETTER_IMPL(ReadStateInterface, JointPosition, _state.qlink)

GENERATE_SETTER_IMPL(WriteStateInterface, JointPosition, _state.qlink)

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointVelocity() const
{
    return derived().impl->_state.vlink;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointVelocity(VecConstRef v)
{
    check_and_set(v, derived().impl->_state.vlink, __func__);
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointAcceleration() const
{
    return derived().impl->_state.a;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointAcceleration(VecConstRef a)
{
    check_and_set(a, derived().impl->_state.a, __func__);
}

template <class Derived>
VecConstRef ReadStateInterface<Derived>::getJointEffort() const
{
    return derived().impl->_state.tau;
}

template<class Derived>
void ReadStateInterface<Derived>::getJointPosition(Eigen::VectorXd &q) const
{
    q = getJointPosition();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointVelocity(Eigen::VectorXd &v) const
{
    v = getJointVelocity();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointAcceleration(Eigen::VectorXd &a) const
{
    a = getJointAcceleration();
}

template<class Derived>
void ReadStateInterface<Derived>::getJointEffort(Eigen::VectorXd &tau) const
{
    tau = getJointEffort();
}

template<class Derived>
std::pair<VecConstRef, VecConstRef> ReadStateInterface<Derived>::getJointLimits() const
{
    return std::make_pair<VecConstRef, VecConstRef>(derived().impl->_state.qmin,
                                                    derived().impl->_state.qmax);
}

template<class Derived>
void ReadStateInterface<Derived>::getJointLimits(Eigen::VectorXd &qmin, Eigen::VectorXd &qmax) const
{
    std::tie(qmin, qmax) = getJointLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getVelocityLimits() const
{
    return derived().impl->_state.vmax;
}

template<class Derived>
void ReadStateInterface<Derived>::getVelocityLimits(Eigen::VectorXd &vmax) const
{
    vmax = getVelocityLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getEffortLimits() const
{
    return derived().impl->_state.taumax;
}

template<class Derived>
void ReadStateInterface<Derived>::getEffortLimits(Eigen::VectorXd &taumax) const
{
    taumax = getVelocityLimits();
}

template<class Derived>
VecConstRef ReadStateInterface<Derived>::getNeutralQ() const
{
    return derived().impl->_state.qneutral;
}

template<class Derived>
void WriteStateInterface<Derived>::setJointEffort(VecConstRef tau)
{
    check_and_set(tau, derived().impl->_state.tau, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setJointLimits(VecConstRef qmin, VecConstRef qmax)
{
    check_and_set(qmin, derived().impl->_state.qmin, __func__);
    check_and_set(qmax, derived().impl->_state.qmax, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setVelocityLimits(VecConstRef vmax)
{
    check_and_set(vmax, derived().impl->_state.vmax, __func__);
}

template<class Derived>
void WriteStateInterface<Derived>::setEffortLimits(VecConstRef taumax)
{
    check_and_set(taumax, derived().impl->_state.taumax, __func__);
}

template<class Derived>
void ReadCmdInterface<Derived>::setPositionReference(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.qcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::POSITION,
                 derived().impl->_cmd.ctrlset);
}

GENERATE_GETTER_IMPL(ReadCmdInterface, MotorPosition, _state.qmot)

GENERATE_GETTER_IMPL(ReadCmdInterface, MotorVelocity, _state.vmot)

GENERATE_MAP_GETTER_IMPL(ReadStateInterface, JointPosition, _state.qlink, q)

GENERATE_MAP_GETTER_IMPL(ReadStateInterface, JointVelocity, _state.vlink, v)

GENERATE_MAP_GETTER_IMPL(ReadStateInterface, JointAcceleration, _state.a, v)

GENERATE_MAP_GETTER_IMPL(ReadStateInterface, JointEffort, _state.tau, v)

GENERATE_MAP_SETTER_IMPL(WriteStateInterface, JointPosition, _state.qlink, Q)

GENERATE_MAP_SETTER_IMPL(WriteStateInterface, JointVelocity, _state.vlink, V)

GENERATE_MAP_SETTER_IMPL(WriteStateInterface, JointAcceleration, _state.a, V)

GENERATE_MAP_SETTER_IMPL(WriteStateInterface, JointEffort, _state.tau, V)


template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getVelocityReference() const
{
    return derived().impl->_cmd.vcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getVelocityReferenceFeedback() const
{
    return derived().impl->_state.vref;
}

template<class Derived>
void ReadCmdInterface<Derived>::setVelocityReference(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.vcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::VELOCITY,
                 derived().impl->_cmd.ctrlset);
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getEffortReference() const
{
    return derived().impl->_cmd.taucmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getEffortReferenceFeedback() const
{
    return derived().impl->_state.tauref;
}

template<class Derived>
void ReadCmdInterface<Derived>::setEffortReference(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.taucmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::EFFORT,
                 derived().impl->_cmd.ctrlset);
}


template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getStiffnessDesired() const
{
    return derived().impl->_cmd.kcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getStiffness() const
{
    return derived().impl->_state.k;
}

template<class Derived>
void ReadCmdInterface<Derived>::setStiffness(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.kcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::STIFFNESS,
                 derived().impl->_cmd.ctrlset);
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getDampingDesired() const
{
    return derived().impl->_cmd.dcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getDamping() const
{
    return derived().impl->_state.d;
}

template<class Derived>
void ReadCmdInterface<Derived>::setDamping(VecConstRef q)
{
    check_and_set(q, derived().impl->_cmd.dcmd, __func__);
    set_ctrlmask(derived().impl->_cmd.ctrlmode,
                 ControlMode::DAMPING,
                 derived().impl->_cmd.ctrlset);
}

template<class Derived>
CtrlModeVectorConstRef ReadCmdInterface<Derived>::getControlMode() const
{
    return derived().impl->_cmd.ctrlmode;
}

template<class Derived>
void ReadCmdInterface<Derived>::getControlMode(CtrlModeMap& ctrl) const
{
    detail::jToMap(derived().impl->_cmd,
                   derived().impl->_cmd.ctrlmode,
                   ctrl);
}

template<class Derived>
void ReadCmdInterface<Derived>::getControlMode(CtrlModeTypeMap& ctrl) const
{
    detail::jToMap(derived().impl->_cmd,
                   derived().impl->_cmd.ctrlmode,
                   ctrl);
}

template<class Derived>
bool ReadCmdInterface<Derived>::setControlMode(CtrlModeVectorConstRef ctrl)
{
    bool success = true;

    static_assert(std::is_same_v<RobotInterface, Derived>||std::is_same_v<RobotJoint, Derived>,
                  "Derived of unexpected class type");

    if constexpr(std::is_same_v<RobotInterface, Derived>)
    {
        for(int i = 0; i < ctrl.size(); i++)
        {
            bool ret = derived().validateControlMode(derived().impl->_cmd.jnames[i],
                                                     static_cast<ControlMode::Type>(ctrl[i]));

            if(ret)
            {
                derived().impl->_cmd.ctrlmode[i] = ctrl[i];
            }
            else
            {
                success = false;
            }
        }
    }
    else if constexpr(std::is_same_v<RobotJoint, Derived>)
    {
        bool ret = derived().validateControlMode(static_cast<ControlMode::Type>(ctrl[0]));

        if(ret)
        {
            derived().impl->_cmd.ctrlmode[0] = ctrl[0];
        }
        else
        {
            success = false;
        }
    }
    else
    {
        // unreachable

    }

    return success;
}

template<class Derived>
bool ReadCmdInterface<Derived>::setControlMode(ControlMode::Type ctrl)
{
    return setControlMode(Eigen::CtrlModeVector::Constant(derived().impl->_cmd.ctrlmode.size(), ctrl));
}

template<class Derived>
bool ReadCmdInterface<Derived>::setControlMode(CtrlModeMap ctrlmap)
{
    auto& cnt = derived().impl->_cmd;
    auto& ctrl = derived().impl->_cmd.ctrlmode;
    bool success = true;

    for(int i = 0; i < cnt.jnames.size(); i++)
    {
        try
        {
            ControlMode::Type ctrl_candidate = ctrlmap.at(cnt.jnames[i]);

            bool ok = false;

            if constexpr(std::is_same_v<RobotInterface, Derived>)
            {
                ok = derived().validateControlMode(cnt.jnames[i],
                                                   ctrl_candidate);
            }
            else if constexpr(std::is_same_v<RobotJoint, Derived>)
            {
                ok = derived().validateControlMode(ctrl_candidate);
            }

            if(ok)
            {
                ctrl[i] = ctrl_candidate;
            }
            else
            {
                success = false;
            }
        }
        catch (std::out_of_range&)
        {

        }
    }

    return success;
}


template<class Derived>
bool ReadCmdInterface<Derived>::setControlMode(CtrlModeTypeMap ctrlmap)
{
    auto& cnt = derived().impl->_cmd;
    auto& ctrl = derived().impl->_cmd.ctrlmode;
    bool success = true;

    for(int i = 0; i < cnt.jnames.size(); i++)
    {
        try
        {
            ControlMode::Type ctrl_candidate = ctrlmap.at(cnt.jnames[i]);

            bool ok = false;

            if constexpr(std::is_same_v<RobotInterface, Derived>)
            {
                ok = derived().validateControlMode(cnt.jnames[i],
                                                   ctrl_candidate);
            }
            else if constexpr(std::is_same_v<RobotJoint, Derived>)
            {
                ok = derived().validateControlMode(ctrl_candidate);
            }

            if(ok)
            {
                ctrl[i] = ctrl_candidate;
            }
            else
            {
                success = false;
            }
        }
        catch (std::out_of_range&)
        {

        }
    }

    return success;
}

template<class Derived>
CtrlModeVectorConstRef ReadCmdInterface<Derived>::getValidCommandMask() const
{
    return derived().impl->_cmd.ctrlset;
}

template<class Derived>
void ReadCmdInterface<Derived>::clearCommandMask()
{
    derived().impl->_cmd.ctrlset.setZero();
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getPositionReference() const
{
    return derived().impl->_cmd.qcmd;
}

template<class Derived>
VecConstRef ReadCmdInterface<Derived>::getPositionReferenceFeedback() const
{
    return derived().impl->_state.qref;
}

template<class Derived>
Derived& ReadStateInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& ReadStateInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& WriteStateInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& WriteStateInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& ReadCmdInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& ReadCmdInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

template<class Derived>
Derived& WriteCmdInterface<Derived>::derived()
{
    return static_cast<Derived&>(*this);
}

template<class Derived>
const Derived& WriteCmdInterface<Derived>::derived() const
{
    return static_cast<const Derived&>(*this);
}

GENERATE_SETTER_IMPL(WriteCmdInterface, MotorPosition, _state.qmot);

GENERATE_SETTER_IMPL(WriteCmdInterface, MotorVelocity, _state.vmot);

template<class Derived>
void WriteCmdInterface<Derived>::setPositionReferenceFeedback(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.qref, __func__);
}


template<class Derived>
void WriteCmdInterface<Derived>::setVelocityReferenceFeedback(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.vref, __func__);
}

template<class Derived>
void WriteCmdInterface<Derived>::setEffortReferenceFeedback(VecConstRef q)
{
    check_and_set(q, derived().impl->_state.tauref, __func__);
}

template<class Derived>
void WriteCmdInterface<Derived>::setStiffnessFeedback(VecConstRef k)
{
    check_and_set(k, derived().impl->_state.k, __func__);
}

template<class Derived>
void WriteCmdInterface<Derived>::setDampingFeedback(VecConstRef k)
{
    check_and_set(k, derived().impl->_state.d, __func__);
}


namespace XBot {

template struct ReadStateInterface<Joint>;
template struct WriteStateInterface<ModelJoint>;
template struct ReadCmdInterface<RobotJoint>;
template struct WriteCmdInterface<UniversalJoint>;

template struct ReadStateInterface<Chain>;

template struct ReadStateInterface<XBotInterface>;
template struct WriteStateInterface<ModelInterface>;
template struct ReadCmdInterface<RobotInterface>;
template struct WriteCmdInterface<RobotInterface>;

template<class Derived>
void ReadCmdInterface<Derived>::getMotorPosition(Eigen::VectorXd &q) const
{
    q = getMotorPosition();
}

template<class Derived>
void ReadCmdInterface<Derived>::getPositionReference(Eigen::VectorXd &q) const
{
    q = getPositionReference();
}

template<class Derived>
void ReadCmdInterface<Derived>::getPositionReferenceFeedback(Eigen::VectorXd &q) const
{
    q = getPositionReferenceFeedback();
}

template<class Derived>
void ReadCmdInterface<Derived>::getMotorVelocity(Eigen::VectorXd &q) const
{
    q = getMotorVelocity();
}

template<class Derived>
void ReadCmdInterface<Derived>::getVelocityReference(Eigen::VectorXd &q) const
{
    q = getVelocityReference();
}

template<class Derived>
void ReadCmdInterface<Derived>::getVelocityReferenceFeedback(Eigen::VectorXd &q) const
{
    q = getVelocityReferenceFeedback();
}

template<class Derived>
void ReadCmdInterface<Derived>::getEffortReference(Eigen::VectorXd &q) const
{
    q = getEffortReference();
}

template<class Derived>
void ReadCmdInterface<Derived>::getEffortReferenceFeedback(Eigen::VectorXd &q) const
{
    q = getEffortReferenceFeedback();
}

template<class Derived>
void ReadCmdInterface<Derived>::getStiffnessDesired(Eigen::VectorXd &q) const
{
    q = getStiffnessDesired();
}

template<class Derived>
void ReadCmdInterface<Derived>::getStiffness(Eigen::VectorXd &q) const
{
    q = getStiffness();
}

template<class Derived>
void ReadCmdInterface<Derived>::getDampingDesired(Eigen::VectorXd &q) const
{
    q = getDampingDesired();
}

template<class Derived>
void ReadCmdInterface<Derived>::getDamping(Eigen::VectorXd &q) const
{
    q = getDamping();
}

}
