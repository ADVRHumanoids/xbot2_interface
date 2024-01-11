#include <fstream>

#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>
#include <xbot2_interface/robotinterface2.h>

#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"
#include "impl/joint.hxx"
#include "impl/load_object.h"

#include <urdf_parser/urdf_parser.h>

using namespace XBot;

XBotInterface::XBotInterface(const XBotInterface::ConfigOptions &opt):
    XBotInterface(opt.urdf, opt.srdf)
{

}

XBotInterface::XBotInterface(urdf::ModelConstSharedPtr urdf,
                             srdf::ModelConstSharedPtr srdf)
{
    if(!urdf)
    {
        throw std::invalid_argument("urdf is null");
    }

    impl = std::make_shared<Impl>(urdf, srdf, *this);
}

XBotInterface::XBotInterface(std::shared_ptr<Impl> _impl):
    impl(_impl)
{

}

std::ostream &XBotInterface::print(std::ostream &os) const
{
    os << "model name: " << getName() << "\n";
    os << "n_joints:   " << getJointNum() << "\n";
    os << "n_q:        " << getNq() << "\n";
    os << "n_v:        " << getNv() << "\n";

    os << "\n";

    os << "joints: \n";

    for(auto j : getJoints())
    {
        auto jinfo = j->getJointInfo();

        os << fmt::format(" - {:<25}: iq = {}\tiv = {}\tnq = {}\tnv = {} \n",
                          j->getName(), jinfo.iq, jinfo.iv, jinfo.nq, jinfo.nv);
    }

    os << "\n";

    os << "model mass: " << getMass() << " kg\n\n";

    os << "q_neutral: " << getNeutralQ().transpose().format(2) << "\n";

    return os;
}

ModelInterface::UniquePtr ModelInterface::getModel(ConfigOptions opt)
{
    std::string type = "pin";

    opt.get_parameter("model_type", type);

    if(const char * model_type_env = getenv("XBOT2IFC_MODEL_TYPE"))
    {
        type = model_type_env;
    }

    auto mdl = CallFunction<ModelInterface*>("libmodelinterface2_" + type + ".so",
                                              "xbot2_create_model_plugin_" + type,
                                              opt);

    mdl->impl->_type = type;

    return UniquePtr(mdl);
}

ModelInterface::UniquePtr ModelInterface::getModel(std::string urdf_string, std::string type)
{
    XBotInterface::ConfigOptions opt;
    opt.set_urdf(urdf_string);

    return getModel(opt.urdf, nullptr, type);
}

ModelInterface::UniquePtr ModelInterface::getModel(std::string urdf_string,
                                   std::string srdf_string,
                                   std::string type)
{
    XBotInterface::ConfigOptions opt;
    opt.set_urdf(urdf_string);
    opt.set_srdf(srdf_string);

    return getModel(opt.urdf, opt.srdf, type);
}

ModelInterface::UniquePtr ModelInterface::getModel(urdf::ModelConstSharedPtr urdf,
                                                   srdf::ModelConstSharedPtr srdf,
                                                   std::string type)
{
    XBotInterface::ConfigOptions opt { urdf, srdf };

    opt.set_parameter("model_type", type);

    return getModel(opt);
}

void ModelInterface::syncFrom(const XBotInterface &other,
                              ControlMode::Type mask)
{
    for(const auto& jname : getJointNames())
    {
        auto oj = other.getJoint(jname);

        if(!oj)
        {
            continue;
        }

        auto j = getJoint(jname);

        if(mask & ControlMode::Type::POSITION)
            j->setJointPosition(oj->getJointPosition());

        if(mask & ControlMode::Type::VELOCITY)
            j->setJointVelocity(oj->getJointVelocity());

        if(mask & ControlMode::Type::EFFORT)
            j->setJointEffort(oj->getJointEffort());

        if(mask & ControlMode::Type::ACCELERATION)
            j->setJointAcceleration(oj->getJointAcceleration());
    }
}

void ModelInterface::syncFrom(const RobotInterface &other,
                              ControlMode::Type mask,
                              Sync flag)
{
    if(flag == Sync::LinkSide)
    {
        syncFrom(static_cast<const XBotInterface&>(other), mask);
        return;
    }

    for(const auto& jname : getJointNames())
    {
        auto oj = other.getJoint(jname);

        if(!oj)
        {
            continue;
        }

        auto j = getJoint(jname);

        if(mask & ControlMode::Type::POSITION)
            j->setJointPosition(oj->getMotorPosition());

        if(mask & ControlMode::Type::VELOCITY)
            j->setJointVelocity(oj->getMotorVelocity());

        if(mask & ControlMode::Type::EFFORT)
            j->setJointEffort(oj->getJointEffort());

        if(mask & ControlMode::Type::ACCELERATION)
            j->setJointAcceleration(oj->getJointAcceleration());
    }
}

void ModelInterface::syncSensors(const XBotInterface &other)
{
    for(const auto& [n, s] : other.getImu())
    {
        try
        {
            getImuNonConst().at(n)->setMeasurement(
                s->getAngularVelocity(),
                s->getLinearAcceleration(),
                s->getOrientation(),
                s->getTimestamp());
        }
        catch(std::out_of_range&)
        {

        }

    }

    for(const auto& [n, s] : other.getForceTorque())
    {
        try
        {
            getForceTorqueNonConst().at(n)->setMeasurement(
                s->getWrench(),
                s->getTimestamp());
        }
        catch(std::out_of_range&)
        {

        }

    }
}

std::string ModelInterface::getType() const
{
    return impl->_type;
}

ModelInterface::UniquePtr ModelInterface::generateReducedModel(
    VecConstRef q, std::vector<std::string> joints_to_fix) const
{
    check_mat_size(q, getNq(), 1, __func__);

    auto urdf = std::make_shared<urdf::Model>();
    urdf->initString(Utils::urdfToString(*getUrdf()));

    // note: not a deep copy
    srdf::ModelSharedPtr srdf;

    if(getSrdf())
    {
        srdf = std::make_shared<srdf::Model>(*getSrdf());
    }

    for(auto jname : joints_to_fix)
    {
        auto uj = urdf->joints_.at(jname);
        auto xj = getJoint(jname);
        auto jinfo = xj->getJointInfo();

        if(uj->type == urdf::Joint::FIXED)
        {
            continue;
        }
        else
        {
            auto& origin = uj->parent_to_joint_origin_transform;

            Eigen::Affine3d Tj;
            Eigen::Vector6d vj;

            double roll, pitch, yaw;
            origin.rotation.getRPY(roll, pitch, yaw);

            xj->forwardKinematics(q.segment(jinfo.iq, jinfo.nq),
                                  Eigen::VectorXd::Zero(jinfo.nv),
                                  Tj, vj);

            Eigen::Quaterniond origin_rot_new(Tj.linear());

            origin.rotation.setFromQuaternion(origin_rot_new.x(),
                                              origin_rot_new.y(),
                                              origin_rot_new.z(),
                                              origin_rot_new.w()
                                              );

            origin.position.x = Tj.translation().x();
            origin.position.y = Tj.translation().y();
            origin.position.z = Tj.translation().z();

            origin.rotation.getRPY(roll, pitch, yaw);

            uj->type = urdf::Joint::FIXED;

        }
    }

    return getModel(urdf, srdf, getType());
}

int ModelInterface::addFixedLink(string_const_ref link_name, string_const_ref parent_name, double mass, Eigen::Matrix3d inertia, Eigen::Affine3d pose)
{
    throw NotImplemented(__PRETTY_FUNCTION__);
}

bool ModelInterface::updateFixedLink(int link_id, double mass, Eigen::Matrix3d inertia, Eigen::Affine3d pose)
{
    throw NotImplemented(__PRETTY_FUNCTION__);
}

void ModelInterface::setJointPositionMinimal(VecConstRef q)
{
    minimalToPosition(q, impl->_state.qlink);
}

void ModelInterface::setJointPositionMinimal(const JointNameMap &qmap)
{
    getJointPositionMinimal(impl->_tmp.v);

    mapToV(qmap, impl->_tmp.v);

    setJointPositionMinimal(impl->_tmp.v);

}

const std::string& XBotInterface::getName() const
{
    return getUrdf()->name_;
}

urdf::ModelConstSharedPtr XBotInterface::getUrdf() const
{
    return impl->_urdf;
}

srdf::ModelConstSharedPtr XBotInterface::getSrdf() const
{
    return impl->_srdf;
}

std::string XBotInterface::getUrdfString() const
{
    return Utils::urdfToString(*getUrdf());
}

std::string XBotInterface::getSrdfString() const
{
    if(!getSrdf())
    {
        throw std::out_of_range("srdf not defined");
    }

    return Utils::srdfToString(*getUrdf(), *getSrdf());
}

XBotInterface::ConfigOptions XBotInterface::getConfigOptions() const
{
    ConfigOptions cfg;
    cfg.urdf = getUrdf();
    cfg.srdf = getSrdf();
    return cfg;
}

bool XBotInterface::hasRobotState(string_const_ref name) const
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

Eigen::VectorXd XBotInterface::getRobotState(string_const_ref name) const
{
    return impl->getRobotState(name);
}

bool XBotInterface::getRobotState(string_const_ref name, Eigen::VectorXd &q) const
{
    try
    {
        q = impl->getRobotState(name);
        return true;
    }
    catch(std::out_of_range&)
    {
        return false;
    }
}

int XBotInterface::getJointNum() const
{
    return impl->_joints.size();
}

int XBotInterface::getActuatedNq() const
{
    auto fb = getJoint(0);

    // very rough, waiting for propert implementation
    if(fb->getType() == urdf::Joint::FLOATING)
    {
        return getNq() - fb->getNq();
    }

    return getNq();
}

int XBotInterface::getActuatedNv() const
{
    auto fb = getJoint(0);

    // very rough, waiting for propert implementation
    if(fb->getType() == urdf::Joint::FLOATING)
    {
        return getNv() - fb->getNv();
    }

    return getNv();
}

bool XBotInterface::hasJoint(string_const_ref name) const
{
    return static_cast<bool>(getJoint(name));
}

ModelJoint::Ptr ModelInterface::getJoint(string_const_ref name)
{
    return impl->getJoint(name);
}

ModelJoint::Ptr ModelInterface::getJoint(int i)
{
    return impl->getJoint(i);
}

ModelJoint::ConstPtr ModelInterface::getJoint(string_const_ref name) const
{
    return impl->getJoint(name);
}

ModelJoint::ConstPtr ModelInterface::getJoint(int i) const
{
    return impl->getJoint(i);
}

const std::vector<ModelJoint::Ptr> &ModelInterface::getJoints()
{
    return impl->_joints_mdl;
}

const std::vector<ModelJoint::ConstPtr> &ModelInterface::getJoints() const
{
    return impl->_joints_mdl_const;
}

void ModelInterface::integrateJointPosition(VecConstRef v)
{
    sum(getJointPosition(), v, impl->_tmp.q);
    setJointPosition(impl->_tmp.q);
}

bool XBot::v2::ModelInterface::getFloatingBasePose(Eigen::Affine3d &w_T_b) const
{
    auto fb = getJoint(0);

    if(fb->getType() != urdf::Joint::FLOATING)
    {
        return false;
    }

    w_T_b = getPose(fb->getChildLink());

    return true;
}

bool ModelInterface::getFloatingBaseTwist(Eigen::Vector6d &v) const
{
    try
    {
        v = getFloatingBaseTwist();
        return true;
    }
    catch (std::runtime_error&)
    {
        return false;
    }
}

Eigen::Affine3d XBot::v2::ModelInterface::getFloatingBasePose() const
{
    auto fb = getJoint(0);

    if(fb->getType() != urdf::Joint::FLOATING)
    {
        throw std::runtime_error("this model is not floating base");
    }

    return getPose(fb->getChildLink());
}

Eigen::Vector6d ModelInterface::getFloatingBaseTwist() const
{
    auto fb = getJoint(0);

    if(fb->getType() != urdf::Joint::FLOATING)
    {
        throw std::runtime_error("this model is not floating base");
    }

    return getVelocityTwist(fb->getChildLink());
}

bool ModelInterface::setFloatingBaseState(const Eigen::Affine3d &w_T_b, const Eigen::Vector6d &twist)
{
    if(!isFloatingBase())
    {
        return false;
    }

    auto fb = getJoint(0);

    // rotate twist to local frame
    Eigen::Vector6d b_vb = twist;
    Utils::rotate(b_vb, w_T_b.linear().transpose());

    Eigen::VectorXd q, v;
    fb->inverseKinematics(w_T_b, b_vb, q, v);
    fb->setJointPosition(q);
    fb->setJointVelocity(v);

    return true;
}

bool ModelInterface::setFloatingBaseState(const ImuSensor &imu)
{
    // pdot = vlin + om x r
    // pddot = alin + dom x t + om x (om x r)

    // pddot = J*a + dJ*v
    // om = J*v

    auto fb = getJoint(0);

    string_const_ref fb_link = getJoint(0)->getUrdfJoint()->child_link_name;

    // set floating base pose rotation part
    // w_T_fb = w_T_imu * imu_T_fb
    Eigen::Matrix3d imu_R_fb = getPose(fb_link, imu.getName()).linear();
    auto T = getFloatingBasePose();
    T.linear() = imu.getOrientation().toRotationMatrix() * imu_R_fb;
    setFloatingBasePose(T);

    // set velocity
    auto J_imu = getJacobian(imu.getName());
    Eigen::Vector6d vfb = fb->getJointVelocity();
    vfb.tail<3>() = J_imu.block<3, 3>(3, 3).fullPivLu().solve(imu.getAngularVelocity());
    fb->setJointVelocity(vfb);

    // tbd acceleration

    return true;

}

bool ModelInterface::setFloatingBasePose(const Eigen::Affine3d& w_T_b)
{
    if(!isFloatingBase())
    {
        return false;
    }

    auto fb = getJoint(0);

    Eigen::VectorXd q, v;
    fb->inverseKinematics(w_T_b, Eigen::Vector6d::Zero(), q, v);
    fb->setJointPosition(q);

    return true;
}

bool ModelInterface::setFloatingBaseOrientation(const Eigen::Matrix3d &w_R_b)
{
    Eigen::Affine3d T;

    if(!getFloatingBasePose(T))
    {
        return false;
    }

    T.linear() = w_R_b;

    return setFloatingBasePose(T);
}

bool ModelInterface::setFloatingBaseTwist(const Eigen::Vector6d &twist)
{
    if(!isFloatingBase())
    {
        return false;
    }

    auto fb = getJoint(0);

    Eigen::VectorXd q, v;
    fb->inverseKinematics(Eigen::Affine3d::Identity(), twist, q, v);
    fb->setJointVelocity(v);

    return true;
}

ModelInterface::~ModelInterface()
{

}

Joint::ConstPtr XBotInterface::getJoint(string_const_ref name) const
{
    return impl->getJoint(name);
}

Joint::ConstPtr XBotInterface::getJoint(int i) const
{
    return impl->getJoint(i);
}

const JointInfo& XBotInterface::getJointInfo(string_const_ref name) const
{
    int jid = getJointId(name);

    if(jid < 0)
    {
        throw std::out_of_range("joint '" + name + "' not found");
    }

    return impl->_joint_info[jid];

}

const JointInfo& XBotInterface::getJointInfo(int i) const
{
    if(i >= getJointNum())
    {
        throw std::out_of_range("joint #'" + std::to_string(i) + "' does not exist");
    }

    return impl->_joint_info[i];
}

int XBotInterface::getJointId(string_const_ref name) const
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

int XBotInterface::getDofIndex(string_const_ref joint_name) const
{
    return getJointInfo(joint_name).iv;
}

int XBotInterface::getQIndex(string_const_ref joint_name) const
{
    return getJointInfo(joint_name).iq;
}

int XBotInterface::getVIndex(string_const_ref joint_name) const
{
    return getJointInfo(joint_name).iv;
}

int XBotInterface::getQIndexFromQName(string_const_ref q_name) const
{
    try
    {
        return impl->_qname_iq_map.at(q_name);
    }
    catch(std::out_of_range&)
    {
        return -1;
    }
}

int XBotInterface::getVIndexFromVName(string_const_ref v_name) const
{
    try
    {
        return impl->_vname_iv_map.at(v_name);
    }
    catch(std::out_of_range&)
    {
        return -1;
    }
}

const std::vector<std::string> &XBotInterface::getJointNames() const
{
    return impl->_joint_name;
}

const std::vector<Joint::Ptr> &XBotInterface::getJoints()
{
    return impl->_joints_xbi;
}

const std::vector<Joint::ConstPtr> &XBotInterface::getJoints() const
{
    return impl->_joints_xbi_const;
}

bool XBotInterface::hasChain(string_const_ref name) const
{
    return impl->_chain_map.contains(name);
}

Chain::ConstPtr XBotInterface::getChain(string_const_ref name) const
{
    return impl->_chain_map.at(name);
}

Chain::Ptr XBotInterface::getChain(string_const_ref name)
{
    return impl->_chain_map.at(name);
}

const std::vector<std::string> &XBotInterface::getChainNames() const
{
    return impl->_chain_names;
}

const std::vector<Chain::Ptr> &XBotInterface::getChains()
{
    return impl->_chains_xbi;
}

const std::vector<Chain::ConstPtr> &XBotInterface::getChains() const
{
    return impl->_chains_xbi_const;
}

void XBotInterface::update()
{
    impl->_tmp.setDirty();
    update_impl();
}

void XBotInterface::qToMap(VecConstRef q, JointNameMap& qmap) const
{
    check_mat_size(q, getNq(), 1, __func__);

    detail::qToMap(impl->_state, q, qmap);
}

void XBotInterface::vToMap(VecConstRef v, JointNameMap& vmap) const
{
    check_mat_size(v, getNv(), 1, __func__);

    detail::vToMap(impl->_state, v, vmap);
}

void XBotInterface::mapToQ(const JointNameMap& qmap, Eigen::VectorXd &q) const
{
    if(q.size() != getNq())
    {
        q = getNeutralQ();
    }

    detail::mapToQ(impl->_state, qmap, q);
}

void XBotInterface::mapToV(const JointNameMap& vmap, Eigen::VectorXd& v) const
{
    if(v.size() != getNv())
    {
        v.setZero(getNv());
    }

    detail::mapToV(impl->_state, vmap, v);
}

bool XBotInterface::checkJointLimits(VecConstRef q) const
{
    auto& dq = impl->_tmp.v;

    difference(q, getNeutralQ(), dq);

    auto& qmin = impl->_state.qmin;
    auto& qmax = impl->_state.qmax;

    return (dq.array() <= qmax.array() && dq.array() >= qmin.array()).all();
}

VecRef XBotInterface::enforceJointLimits(Eigen::VectorXd& q, double tol) const
{
    check_mat_size(q, getNq(), 1, __func__);

    auto& dq = impl->_tmp.v;
    difference(q, getNeutralQ(), dq);

    auto& qmin = impl->_state.qmin;
    auto& qmax = impl->_state.qmax;
    dq = dq.array().max(qmin.array() + tol).min(qmax.array() - tol);

    sum(getNeutralQ(), dq, q);

    return q;

}

void XBotInterface::generateRandomQ(Eigen::VectorXd &qrand) const
{
    auto& dq = impl->_tmp.v;
    auto& qmin = impl->_state.qmin;
    auto& qmax = impl->_state.qmax;

    // set random motion between -1 and 1
    dq.setRandom();

    // hack!!!
    for(int i = 0; i < getJointNum(); i++)
    {
        auto j = getJoint(i);

        if(j->getType() == urdf::Joint::FLOATING)
        {
            dq.segment<3>(j->getJointInfo().iv) /= M_PI;
        }
    }

    // map to [0, 1]
    dq = (dq.array() + 1)/2.;

    // map to [qmin, qmax]
    dq = dq.cwiseProduct(qmax - qmin) + qmin;

    // apply motion
    qrand.resize(getNq());
    sum(impl->_state.qneutral, dq, qrand);
}

Eigen::VectorXd XBotInterface::generateRandomQ() const
{
    Eigen::VectorXd qrand;
    generateRandomQ(qrand);
    return qrand;
}

std::map<std::string, ImuSensor::ConstPtr> XBotInterface::getImu() const
{
    std::map<std::string, ImuSensor::ConstPtr> ret(impl->_imu_map.begin(),
                                                   impl->_imu_map.end());

    return ret;
}

ImuSensor::ConstPtr XBotInterface::getImu(string_const_ref name) const
{
    try
    {
        return impl->_imu_map.at(name);
    }
    catch(std::out_of_range&)
    {
        return nullptr;
    }
}

std::map<std::string, ForceTorqueSensor::ConstPtr> XBotInterface::getForceTorque() const
{
    std::map<std::string, ForceTorqueSensor::ConstPtr> ret(impl->_ft_map.begin(),
                                                           impl->_ft_map.end());

    return ret;
}

ForceTorqueSensor::ConstPtr XBotInterface::getForceTorque(string_const_ref name) const
{
    try
    {
        return impl->_ft_map.at(name);
    }
    catch(std::out_of_range&)
    {
        return nullptr;
    }
}

bool XBotInterface::addSensor(Sensor::Ptr s)
{
    if(impl->_sensor_map.contains(s->getName()))
    {
        return false;
    }

    impl->_sensor_map[s->getName()] = s;

    if(auto ft = std::dynamic_pointer_cast<ForceTorqueSensor>(s))
    {
        impl->_ft_map[s->getName()] = ft;
    }
    else if(auto imu = std::dynamic_pointer_cast<ImuSensor>(s))
    {
        impl->_imu_map[s->getName()] =  imu;
    }

    return true;
}

void XBotInterface::getJointPositionMinimal(Eigen::VectorXd &q) const
{
    positionToMinimal(getJointPosition(), q);
}

void XBotInterface::getJointPositionMinimal(JointNameMap &q) const
{
    getJointPositionMinimal(impl->_tmp.v);
    vToMap(impl->_tmp.v, q);
}

Eigen::VectorXd XBotInterface::getJointPositionMinimal() const
{
    Eigen::VectorXd ret;

    getJointPositionMinimal(ret);

    return ret;
}

void XBotInterface::minimalToPosition(VecConstRef q_minimal, VecRef q) const
{
    check_mat_size(q_minimal, getNv(), 1, __func__);
    check_mat_size(q, getNq(), 1, __func__);

    for(auto& j : impl->_joints)
    {
        j->minimalToPosition(q_minimal.segment(j->getVIndex(), j->getNv()),
                             q.segment(j->getQIndex(), j->getNq()));
    }
}

void XBotInterface::minimalToPosition(VecConstRef q_minimal, Eigen::VectorXd &q) const
{
    q.resize(getNq());
    minimalToPosition(q_minimal, VecRef(q));
}

void XBotInterface::minimalToPosition(const JointNameMap &q_minimal,
                                      Eigen::VectorXd &q) const
{
    positionToMinimal(q, impl->_tmp.v);
    mapToV(q_minimal, impl->_tmp.v);
    minimalToPosition(impl->_tmp.v, q);
}

Eigen::VectorXd XBotInterface::minimalToPosition(VecConstRef q_minimal) const
{
    Eigen::VectorXd ret;
    minimalToPosition(q_minimal, ret);
    return ret;
}

void XBotInterface::positionToMinimal(VecConstRef q, VecRef q_minimal) const
{
    check_mat_size(q_minimal, getNv(), 1, __func__);
    check_mat_size(q, getNq(), 1, __func__);

    for(auto& j : impl->_joints)
    {
        j->positionToMinimal(q.segment(j->getQIndex(), j->getNq()),
                             q_minimal.segment(j->getVIndex(), j->getNv()));
    }
}

void XBotInterface::positionToMinimal(VecConstRef q, JointNameMap &q_minimal) const
{
    positionToMinimal(q, impl->_tmp.v);
    vToMap(impl->_tmp.v, q_minimal);
}

void XBotInterface::positionToMinimal(VecConstRef q,
                                      Eigen::VectorXd &q_minimal) const
{
    q_minimal.resize(getNv());
    positionToMinimal(q, VecRef(q_minimal));
}

Eigen::VectorXd XBotInterface::positionToMinimal(VecConstRef q) const
{
    Eigen::VectorXd ret;
    positionToMinimal(q, ret);
    return ret;
}

bool XBotInterface::isFloatingBase() const
{
    return getJoint(0)->getType() == urdf::Joint::FLOATING;
}

bool XBotInterface::getFloatingBaseLink(std::string &fb) const
{
    if(!isFloatingBase())
    {
        return false;
    }

    fb = getJoint(0)->getUrdfJoint()->child_link_name;

    return true;
}

string_const_ref XBotInterface::getFloatingBaseLink() const
{
    if(!isFloatingBase())
    {
        throw std::runtime_error("model is not floating base");
    }

    return getJoint(0)->getUrdfJoint()->child_link_name;
}



bool XBotInterface::getJacobian(string_const_ref link_name, MatRef J) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    getJacobian(idx, J);

    return true;
}

bool XBotInterface::getJacobian(string_const_ref link_name, Eigen::MatrixXd &J) const
{
    J.setZero(6, getNv());
    return getJacobian(link_name, MatRef(J));
}

bool XBotInterface::getJacobian(string_const_ref link_name,
                                const Eigen::Vector3d &p,
                                Eigen::MatrixXd &J) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    J.setZero(6, getNv());

    getJacobian(idx, p, J);

    return true;
}

void XBotInterface::getJacobian(int link_id,
                                const Eigen::Vector3d &p,
                                Eigen::MatrixXd &J) const
{
    getJacobian(link_id, J);

    Eigen::Vector3d p_world = getPose(link_id).linear()*p;

    Utils::changeRefPoint(J, p_world);
}

Eigen::MatrixXd XBotInterface::getJacobian(string_const_ref link_name) const
{
    Eigen::MatrixXd J;

    J.setZero(6, getNv());

    int idx = impl->get_link_id_throw(link_name);

    getJacobian(idx, J);

    return J;
}


void XBotInterface::getRelativeJacobian(int distal_id, int base_id, MatRef Jrel) const
{
    // check mat size
    check_mat_size(Jrel, 6, getNv(), __func__);

    // take Jrel from tmp
    auto& Jtmp = impl->_tmp.J;
    auto& Jtmp1 = impl->_tmp.J1;

    // initialize Jrel with Jbase
    getJacobian(base_id, Jtmp);

    // shift ref point to p_d
    Eigen::Affine3d w_T_b = getPose(base_id);
    Eigen::Affine3d w_T_d = getPose(distal_id);
    Eigen::Vector3d r = w_T_d.translation() - w_T_b.translation();
    Utils::changeRefPoint(Jtmp, r);

    // get distal jacobian
    getJacobian(distal_id, Jrel);

    // now Jrel = Jd - Jb_shifted
    Jtmp1 = Jrel - Jtmp;

    // rotate to base
    Utils::rotate(Jtmp1, w_T_b.linear().transpose(), Jrel);
}

bool XBotInterface::getRelativeJacobian(string_const_ref distal_name,
                                        string_const_ref base_name,
                                        MatRef J) const
{
    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    J.setZero();

    getRelativeJacobian(didx, bidx, J);

    return true;
}

bool XBotInterface::getRelativeJacobian(string_const_ref distal_name,
                                         string_const_ref base_name,
                                         Eigen::MatrixXd &J) const
{

    J.setZero(6, getNv());

    getRelativeJacobian(distal_name, base_name, MatRef(J));

    return true;

}

Eigen::MatrixXd XBotInterface::getRelativeJacobian(string_const_ref distal_name,
                                                   string_const_ref base_name) const
{
    Eigen::MatrixXd J;

    J.setZero(6, getNv());

    int didx = impl->get_link_id_throw(distal_name);

    int bidx = impl->get_link_id_throw(base_name);

    getRelativeJacobian(didx, bidx, J);

    return J;
}

Eigen::Affine3d XBotInterface::getPose(string_const_ref link_name) const
{
    return getPose(impl->get_link_id_throw(link_name));
}

bool XBotInterface::getPose(string_const_ref link_name, Eigen::Affine3d &w_T_l) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    w_T_l = getPose(idx);

    return true;
}

bool XBotInterface::getOrientation(string_const_ref link_name,
                                   Eigen::Matrix3d &w_R_l) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    w_R_l = getPose(idx).linear();

    return true;
}

Eigen::Affine3d XBotInterface::getPose(int distal_id, int base_id) const
{
    return getPose(base_id).inverse() * getPose(distal_id);
}

Eigen::Affine3d XBotInterface::getPose(string_const_ref distal_name, string_const_ref base_name) const
{
    return getPose(base_name).inverse() * getPose(distal_name);
}

bool XBotInterface::getPose(string_const_ref distal_name, string_const_ref base_name, Eigen::Affine3d &w_T_l) const
{
    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    w_T_l = getPose(didx, bidx);

    return true;
}

bool XBotInterface::getOrientation(string_const_ref distal_name,
                                   string_const_ref base_name,
                                   Eigen::Affine3d &b_R_l) const
{
    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    b_R_l = getPose(didx, bidx).linear();

    return true;
}

Eigen::Vector6d XBotInterface::getVelocityTwist(int link_id) const
{
    Eigen::Vector6d ret;
    getJacobian(link_id, impl->_tmp.J);
    ret.noalias() = impl->_tmp.J * getJointVelocity();
    return ret;
}

Eigen::Vector6d XBotInterface::getVelocityTwist(string_const_ref link_name) const
{
    return getVelocityTwist(impl->get_link_id_throw(link_name));
}

bool XBotInterface::getVelocityTwist(string_const_ref link_name, Eigen::Vector6d &v) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    v = getVelocityTwist(idx);

    return true;
}

Eigen::Vector6d XBotInterface::getAccelerationTwist(int link_id) const
{
    Eigen::Vector6d ret;
    getJacobian(link_id, impl->_tmp.J);
    ret.noalias() = impl->_tmp.J*getJointAcceleration() + getJdotTimesV(link_id);
    return ret;
}

Eigen::Vector6d XBotInterface::getAccelerationTwist(string_const_ref link_name) const
{
    return getAccelerationTwist(impl->get_link_id_throw(link_name));
}

bool XBotInterface::getAccelerationTwist(string_const_ref link_name, Eigen::Vector6d &v) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    v = getAccelerationTwist(idx);

    return true;
}

Eigen::Vector6d XBotInterface::getJdotTimesV(int link_id) const
{
    throw std::runtime_error(__func__ + std::string(" not implemented by base class"));
}

Eigen::Vector6d XBotInterface::getJdotTimesV(string_const_ref link_name) const
{
    return getJdotTimesV(impl->get_link_id_throw(link_name));
}

bool XBotInterface::getJdotTimesV(string_const_ref link_name, Eigen::Vector6d &a) const
{
    int idx = impl->get_link_id_error(link_name);

    if(idx < 0)
    {
        return false;
    }

    a = getJdotTimesV(idx);

    return true;
}

Eigen::Vector6d XBotInterface::getRelativeVelocityTwist(int distal_id, int base_id) const
{
    // initialize vrel with Jbase
    Eigen::Vector6d vrel = getVelocityTwist(base_id);

    // shift ref point to p_d
    Eigen::Affine3d w_T_b = getPose(base_id);
    Eigen::Affine3d w_T_d = getPose(distal_id);
    Eigen::Vector3d r = w_T_d.translation() - w_T_b.translation();
    Utils::changeRefPoint(vrel, r);

    // now Jrel = Jd - Jb_shifted
    vrel = getVelocityTwist(distal_id) - vrel;

    // rotate to base
    Utils::rotate(vrel, w_T_b.linear().transpose());

    return vrel;
}

Eigen::Vector6d XBotInterface::getRelativeVelocityTwist(string_const_ref distal_name,
                                                         string_const_ref base_name) const
{
    return getRelativeVelocityTwist(impl->get_link_id_throw(distal_name),
                                    impl->get_link_id_throw(base_name));
}

bool XBotInterface::getRelativeVelocityTwist(string_const_ref distal_name,
                                              string_const_ref base_name,
                                              Eigen::Vector6d &v) const
{
    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    v = getRelativeVelocityTwist(didx, bidx);

    return true;
}

Eigen::Vector6d XBotInterface::getRelativeAccelerationTwist(int distal_id, int base_id) const
{
    Eigen::Vector6d v_base = getVelocityTwist(base_id);
    Eigen::Vector6d v_distal = getVelocityTwist(distal_id);

    Eigen::Vector6d v_rel = getRelativeVelocityTwist(distal_id, base_id);

    Eigen::Affine3d w_T_b = getPose(base_id);
    Eigen::Affine3d w_T_d = getPose(distal_id);

    Eigen::Vector6d a_d = getAccelerationTwist(distal_id);
    Eigen::Vector6d a_b = getAccelerationTwist(base_id);

    const Eigen::Matrix3d& w_R_b = w_T_b.linear();
    Eigen::Vector3d r = w_T_d.translation() - w_T_b.translation();
    Eigen::Vector3d b_om_base = w_R_b.transpose() * v_base.tail<3>();
    Eigen::Vector3d rdot = (v_distal - v_base).head<3>();

    Utils::rotate(v_rel, Utils::skew(b_om_base));
    Utils::changeRefPoint(a_b, r);

    Eigen::Vector6d rdot_cross_om;
    rdot_cross_om << rdot.cross(v_base.tail<3>()), 0, 0, 0;

    Eigen::Vector6d aux = a_d - a_b + rdot_cross_om;

    return -v_rel + Utils::rotate(aux, w_R_b.transpose());

}

Eigen::Vector6d XBotInterface::getRelativeAccelerationTwist(string_const_ref distal_name,
                                                             string_const_ref base_name) const
{
    return getRelativeAccelerationTwist(impl->get_link_id_throw(distal_name),
                                        impl->get_link_id_throw(base_name));
}

bool XBotInterface::getRelativeAccelerationTwist(string_const_ref distal_name, string_const_ref base_name, Eigen::Vector6d &v) const
{
    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    v = getRelativeAccelerationTwist(didx, bidx);

    return true;
}

bool XBotInterface::getCOMJacobian(Eigen::MatrixXd &J) const
{
    J.setZero(3, getNv());
    getCOMJacobian(MatRef(J));
    return true;
}

Eigen::MatrixXd XBotInterface::getCOMJacobian() const
{
    Eigen::MatrixXd Jcom;
    Jcom.setZero(3, getNv());
    getCOMJacobian(Jcom);
    return Jcom;
}

Eigen::Vector3d XBotInterface::getCOMVelocity() const
{
    getCOMJacobian(impl->_tmp.J);
    Eigen::Vector3d ret;
    ret.noalias() = impl->_tmp.J * getJointVelocity();
    return ret;
}

Eigen::Vector6d XBotInterface::getRelativeJdotTimesV(int distal_id, int base_id) const
{
    Eigen::Vector6d v_base = getVelocityTwist(base_id);
    Eigen::Vector6d v_distal = getVelocityTwist(distal_id);

    Eigen::Vector6d v_rel = getRelativeVelocityTwist(distal_id, base_id);

    Eigen::Affine3d w_T_b = getPose(base_id);
    Eigen::Affine3d w_T_d = getPose(distal_id);

    Eigen::Vector6d a0_d = getJdotTimesV(distal_id);
    Eigen::Vector6d a0_b = getJdotTimesV(base_id);

    const Eigen::Matrix3d& w_R_b = w_T_b.linear();
    Eigen::Vector3d r = w_T_d.translation() - w_T_b.translation();
    Eigen::Vector3d b_om_base = w_R_b.transpose() * v_base.tail<3>();
    Eigen::Vector3d rdot = (v_distal - v_base).head<3>();

    Utils::rotate(v_rel, Utils::skew(b_om_base));
    Utils::changeRefPoint(a0_b, r);

    Eigen::Vector6d rdot_cross_om;
    rdot_cross_om << rdot.cross(v_base.tail<3>()), 0, 0, 0;

    Eigen::Vector6d aux = a0_d - a0_b + rdot_cross_om;

    return -v_rel + Utils::rotate(aux, w_R_b.transpose());

}

Eigen::Vector6d XBotInterface::getRelativeJdotTimesV(string_const_ref distal_name,
                                                      string_const_ref base_name) const
{
    return getRelativeJdotTimesV(impl->get_link_id_throw(distal_name),
                                 impl->get_link_id_throw(base_name));
}

bool XBotInterface::getRelativeJdotTimesV(string_const_ref distal_name,
                                           string_const_ref base_name,
                                           Eigen::Vector6d &v) const
{

    int didx = impl->get_link_id_error(distal_name);

    int bidx = impl->get_link_id_error(base_name);

    if(didx < 0 || bidx < 0)
    {
        return false;
    }

    v = getRelativeJdotTimesV(didx, bidx);

    return true;
}

void XBotInterface::computeInverseDynamics(Eigen::VectorXd& rnea) const
{
    rnea = computeInverseDynamics();
}

void XBotInterface::computeGravityCompensation(Eigen::VectorXd& gcomp) const
{
    gcomp = computeGravityCompensation();
}

void XBotInterface::computeNonlinearTerm(Eigen::VectorXd &h) const
{
    h = computeNonlinearTerm();
}

MatConstRef XBotInterface::computeInertiaInverse() const
{
    throw NotImplemented(__func__);
}

void XBotInterface::computeCentroidalMomentumMatrix(Eigen::MatrixXd &Ag) const
{
    Ag = computeCentroidalMomentumMatrix();
}

Eigen::Vector6d XBotInterface::computeCentroidalMomentum() const
{
    Eigen::Vector6d ret;
    ret.noalias() = computeCentroidalMomentumMatrix() * getJointVelocity();
    return ret;
}

void XBotInterface::computeForwardDynamics(Eigen::VectorXd &fd) const
{
    fd = computeForwardDynamics();
}

void XBotInterface::computeInertiaMatrix(Eigen::MatrixXd &M) const
{
    M = computeInertiaMatrix();
}

void XBotInterface::computeInertiaInverse(Eigen::MatrixXd &Minv) const
{
    Minv = computeInertiaInverse();
}

Eigen::VectorXd XBotInterface::sum(VecConstRef q0, VecConstRef v) const
{
    Eigen::VectorXd q1;
    sum(q0, v, q1);
    return q1;
}

Eigen::VectorXd XBotInterface::difference(VecConstRef q1, VecConstRef q0) const
{
    Eigen::VectorXd v;
    difference(q1, q0, v);
    return v;
}

XBotInterface::JointParametrization XBotInterface::get_joint_parametrization(string_const_ref jname)
{
    return impl->get_joint_parametrization(jname);
}

std::map<std::string, ImuSensor::Ptr> XBotInterface::getImuNonConst()
{
    return impl->_imu_map;
}

std::map<std::string, ForceTorqueSensor::Ptr> XBotInterface::getForceTorqueNonConst()
{
    return impl->_ft_map;
}

UniversalJoint::Ptr XBotInterface::getUniversalJoint(string_const_ref name)
{
    return impl->getJoint(name);
}

UniversalJoint::Ptr XBotInterface::getUniversalJoint(int i)
{
    return impl->getJoint(i);
}

void XBotInterface::finalize()
{
    return impl->finalize();
}

XBotInterface::~XBotInterface()
{

}

// impl
XBotInterface::Impl::Impl(urdf::ModelConstSharedPtr urdf,
                           srdf::ModelConstSharedPtr srdf,
                           XBotInterface& api):
    _api(&api),
    _urdf(urdf),
    _srdf(srdf)
{
    parse_imu();

    parse_ft();
}

Eigen::VectorXd XBotInterface::Impl::getRobotState(string_const_ref name) const
{
    if(!_srdf)
    {
        throw std::out_of_range("cannot retrieve robot state: no srdf defined");
    }

    Eigen::VectorXd qrs = _state.qneutral;

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

UniversalJoint::Ptr XBotInterface::Impl::getJoint(string_const_ref name) const
{
    auto it = _name_id_map.find(name);

    if(it == _name_id_map.end())
    {
        return nullptr;
    }

    int i = it->second;
    return _joints.at(i);
}

UniversalJoint::Ptr XBotInterface::Impl::getJoint(int i) const
{
    return _joints.at(i);
}

XBotInterface::JointParametrization
XBotInterface::Impl::get_joint_parametrization(string_const_ref)
{
    JointParametrization ret;

    throw std::runtime_error(std::string(__func__) + " not impl");

    return ret;
}

int XBotInterface::Impl::get_link_id_error(string_const_ref link_name) const
{
    int idx = _api->getLinkId(link_name);

    if(idx < 0)
    {
        std::cerr << "link '" << link_name << "' does not exist \n";
    }

    return idx;
}

int XBotInterface::Impl::get_link_id_throw(string_const_ref link_name) const
{
    int idx = _api->getLinkId(link_name);

    if(idx < 0)
    {
        std::ostringstream ss;
        ss << "link '" << link_name << "' does not exist \n";
        throw std::out_of_range(ss.str());
    }

    return idx;
}

void XBotInterface::Impl::finalize()
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
        auto jparam = _api->get_joint_parametrization(jname);

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
        auto [jid, jiq, jiv, jnq, jnv, passive] = _joint_info[i];

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

    // set neutral config
    for(int i = 0; i < nj; i++)
    {
        _state.qneutral.segment(_joint_info[i].iq,
                                _joint_info[i].nq) = jparam_map[_joint_name[i]].q0;
    }

    // use it to initialize cmd and state
    _cmd.qcmd = _state.qref = _state.qmot = _state.qlink = _state.qneutral;

    // construct internal joints
    for(int i = 0; i < nj; i++)
    {
        auto jname = _joint_name[i];
        auto jptr = _urdf->joints_.at(jname);

        auto [id, iq, iv, nq, nv, passive] = _joint_info[i];

        // assign joint qname and vname
        if(nq == 1)
        {
            _cmd.qnames[iq] = _state.qnames[iq] = jname;
            _qname_iq_map[_cmd.qnames[iq]] = iq;
        }
        else
        {
            for(int i = 0; i < nq; i++)
            {
                _cmd.qnames[iq + i] = _state.qnames[iq + i] = jname + "@q" + std::to_string(i);
                _qname_iq_map[_cmd.qnames[iq + i]] = iq + i;
            }
        }

        if(nv == 1)
        {
            _cmd.vnames[iv] = _state.vnames[iv] = jname;
            _vname_iv_map[_cmd.vnames[iv]] = iv;
        }
        else
        {
            for(int i = 0; i < nv; i++)
            {
                _cmd.vnames[iv + i] = _state.vnames[iv + i] = jname + "@v" + std::to_string(i);
                _vname_iv_map[_cmd.vnames[iv + i]] = iv + i;
            }
        }

        _cmd.jnames[i] = jname;

        // create xbot's joint

        // get views on relevant states and control
        auto sv = detail::createView(_state,
                                     iq, nq,
                                     iv, nv,
                                     i, 1);

        auto cv = detail::createView(_cmd,
                                     iq, nq,
                                     iv, nv,
                                     i, 1);

        // check if this joint is passive from srdf
        if(_srdf)
        {
            auto it = std::find_if(_srdf->getPassiveJoints().begin(),
                                   _srdf->getPassiveJoints().end(),
                                   [jname](const auto &item) { return item.name_ == jname; });

            _joint_info[i].passive = (it != _srdf->getPassiveJoints().end());
        }
        else
        {
            _joint_info[i].passive = false;
        }

        // hack? floating joints are passive by default
        if(jptr->type == urdf::Joint::FLOATING)
        {
            _joint_info[i].passive = true;
        }

        // create private implementation of joint
        auto jimpl = std::make_unique<Joint::Impl>(sv, cv, jptr, _joint_info[i]);

        // inject mappings
        auto& jparam = jparam_map.at(jname);
        jimpl->fn_minimal_to_q = jparam.fn_minimal_to_q;
        jimpl->fn_q_to_minimal = jparam.fn_q_to_minimal;

        if(jparam.fn_fwd_kin)
        {
            jimpl->fn_fwd_kin = jparam.fn_fwd_kin;
        }

        if(jparam.fn_inv_kin)
        {
            jimpl->fn_inv_kin = jparam.fn_inv_kin;
        }

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
        // in several flavours to be returned by reference by models and robots
        auto j =  std::make_shared<UniversalJoint>(std::move(jimpl));
        _joints.push_back(j);
        _joints_xbi.push_back(j);
        _joints_xbi_const.push_back(j);
        _joints_mdl.push_back(j);
        _joints_mdl_const.push_back(j);
        _joints_rob.push_back(j);
        _joints_rob_const.push_back(j);

        // set joint limits
        auto lims = jptr->limits;

        double infinity = 1e3;

        if(jptr->type == urdf::Joint::REVOLUTE ||
            jptr->type == urdf::Joint::PRISMATIC)
        {
            _state.qmin[iv] = lims ? lims->lower : -infinity;
            _state.qmax[iv] = lims ? lims->upper : infinity;
            _state.vmax[iv] = lims ? lims->velocity : infinity;
            _state.taumax[iv] = lims ? lims->effort : infinity;
        }

        if(jptr->type == urdf::Joint::CONTINUOUS)
        {
            _state.qmin[iv] = -infinity;
            _state.qmax[iv] = infinity;
            _state.vmax[iv] = lims ? lims->velocity : infinity;
            _state.taumax[iv] = lims ? lims->effort : infinity;
        }

        if(jptr->type == urdf::Joint::FLOATING)
        {
            _state.qmin.segment<3>(iv).setConstant(-infinity);
            _state.qmax.segment<3>(iv).setConstant(infinity);
            _state.qmin.segment<3>(iv+3).setConstant(-M_PI);
            _state.qmax.segment<3>(iv+3).setConstant(M_PI);
            _state.vmax.segment<6>(iv).setConstant(lims ? lims->velocity : infinity);
            _state.taumax.segment<6>(iv).setConstant(lims ? lims->effort : infinity);
        }


    }

    // parse chains
    if(_srdf)
    {
        parse_chains();
    }

    // resize temporaries
    _tmp.setZero(nq, nv);

    // trigger model update
    _api->update();

}

void XBotInterface::Impl::parse_imu()
{
    if(!_srdf)
    {
        return;
    }

    auto groups = _srdf->getGroups();

    for(auto& g : groups)
    {
        if(g.name_ != "imu_sensors")
        {
            continue;
        }

        for(auto lname : g.links_)
        {
            _sensor_map[lname] = _imu_map[lname] =
                std::make_shared<ImuSensor>(lname);
        }
    }
}

void XBotInterface::Impl::parse_chains()
{
    srdf::Model::Group chains;

    for(auto g : _srdf->getGroups())
    {
        if(g.name_ == "chains")
        {
            chains = g;
        }
    }

    if(chains.name_ != "chains")
    {
        return;
    }

    for(auto sg : chains.subgroups_)
    {
        auto it = std::find_if(_srdf->getGroups().begin(),
                               _srdf->getGroups().end(),
                               [sg](const auto& elem) { return elem.name_ == sg; }
                               );

        if(it->chains_.size() != 1)
        {
            throw std::runtime_error("group '" + sg + "' must contain one single chain");
        }

        // get base link and tip link

        auto [base_link, tip_link] = it->chains_[0];

        auto link = _urdf->getLink(tip_link);


        // assign all joints belonging to this chain

        std::list<UniversalJoint::Ptr> chain_joints;
        int chain_nq = 0;
        int chain_nv = 0;

        while(link->name != base_link)
        {
            auto j = link->parent_joint;

            if(!j)
            {
                throw std::out_of_range("got null parent joint from link '" + link->name +
                                        "' while forming chain '" + sg + "'");
            }

            if(j->type != urdf::Joint::FIXED)
            {
                auto it = std::find_if(_joints.begin(), _joints.end(),
                                       [j](const auto& item)
                                       {
                                           return item->getName() == j->name;
                                       });

                if(it == _joints.end())
                {
                    throw std::out_of_range(
                        "could not get joint '" + j->name +
                        "' while forming chain '" + sg + "'");
                }

                chain_joints.push_front(*it);

                chain_nq += (**it).getNq();

                chain_nv += (**it).getNv();
            }

            link = _urdf->getLink(j->parent_link_name);

            if(!link)
            {
                throw std::out_of_range("got null parent link from joint '" + j->name +
                                        "' while forming chain '" + sg + "'");
            }
        }

        // make state view
        auto chain_state = detail::createView(_state,
                                              chain_joints.front()->getQIndex(),
                                              chain_nq,
                                              chain_joints.front()->getVIndex(),
                                              chain_nv,
                                              chain_joints.front()->getId(),
                                              chain_joints.size());

        auto chain_cmd = detail::createView(_cmd,
                                            chain_joints.front()->getQIndex(),
                                            chain_nq,
                                            chain_joints.front()->getVIndex(),
                                            chain_nv,
                                            chain_joints.front()->getId(),
                                            chain_joints.size());

        auto chain_impl = std::make_unique<Chain::Impl>(
            chain_state, chain_cmd, sg, base_link, tip_link, chain_joints
            );

        auto chain = std::make_shared<Chain>(std::move(chain_impl));

        _chain_map[chain->getName()] = chain;

        _chain_names.push_back(chain->getName());

        _chains_xbi.push_back(chain);

        _chains_xbi_const.push_back(chain);

    }
}

void XBotInterface::Impl::parse_ft()
{
    if(!_srdf)
    {
        return;
    }

    auto groups = _srdf->getGroups();

    for(auto& g : groups)
    {
        if(g.name_ != "force_torque_sensors")
        {
            continue;
        }

        for(auto lname : g.links_)
        {
            _sensor_map[lname] = _ft_map[lname] =
                std::make_shared<ForceTorqueSensor>(lname);
        }
    }
}

void XBotInterface::Impl::Temporaries::setZero(int nq, int nv)
{
    J1 = J.setZero(6, nv);
    Jarg.setZero(6, nv);
    v.setZero(nv);
    q.setZero(nq);
    M.setIdentity(nv, nv);
    ldlt.compute(M);
}

void XBotInterface::Impl::Temporaries::setDirty()
{
    ldlt_dirty = true;
}

bool XBotInterface::ConfigOptions::set_urdf(std::string urdf_string)
{
    auto ncurdf = std::make_shared<urdf::Model>();
    urdf = ncurdf;
    return ncurdf->initString(urdf_string);
}

bool XBotInterface::ConfigOptions::set_srdf(std::string srdf_string)
{
    auto ncsrdf = std::make_shared<srdf::Model>();
    srdf = ncsrdf;
    return ncsrdf->initString(*urdf, srdf_string);
}

bool ConfigOptions::set_urdf_path(std::string urdf_path)
{
    std::ifstream f(urdf_path);

    if(f.fail())
    {
        return false;
    }

    std::stringstream ss;

    ss << f.rdbuf();

    return set_urdf(ss.str());
}

bool ConfigOptions::set_srdf_path(std::string srdf_path)
{
    std::ifstream f(srdf_path);

    if(f.fail())
    {
        return false;
    }

    std::stringstream ss;

    ss << f.rdbuf();

    return set_srdf(ss.str());
}
