#include <xbot2_interface/common/plugin.h>
#include <xbot2_interface/common/utils.h>

#include "impl/xbotinterface2.hxx"
#include "impl/utils.h"
#include "impl/joint.hxx"
#include "impl/load_object.h"

using namespace XBot;

XBotInterface::XBotInterface(const XBotInterface::ConfigOptions &opt):
    XBotInterface(opt.urdf, opt.srdf)
{

}

XBotInterface::XBotInterface(urdf::ModelConstSharedPtr urdf,
                             srdf::ModelConstSharedPtr srdf)
{
    impl = std::make_shared<Impl>(urdf, srdf, *this);
}

XBotInterface::XBotInterface(std::shared_ptr<Impl> _impl):
    impl(_impl)
{

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

    auto mdl = CallFunction<ModelInterface*>("libmodelinterface2_" + type + ".so",
                                              "xbot2_create_model_plugin_" + type,
                                              opt);

    mdl->impl->_type = type;

    return UniquePtr(mdl);
}

void ModelInterface::syncFrom(const XBotInterface &other)
{
    for(const auto& jname : getJointNames())
    {
        auto oj = other.getJoint(jname);

        if(!oj)
        {
            continue;
        }

        auto j = getJoint(jname);

        j->setJointPosition(oj->getJointPosition());
        j->setJointVelocity(oj->getJointVelocity());
        j->setJointEffort(oj->getJointEffort());
        j->setJointAcceleration(oj->getJointAcceleration());
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

    auto urdf = std::make_shared<urdf::Model>(*getUrdf());
    auto srdf = std::make_shared<srdf::Model>(*getSrdf());

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

bool ModelInterface::setFloatingBaseState(const Eigen::Affine3d &w_T_b, const Eigen::Vector6d &twist)
{
    if(!isFloatingBase())
    {
        return false;
    }

    auto fb = getJoint(0);

    Eigen::VectorXd q, v;
    fb->inverseKinematics(w_T_b, twist, q, v);
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

    auto Jimu = getJacobian(imu.getName());

    Eigen::Vector6d vfb = fb->getJointVelocity();
    vfb.tail<3>() = Jimu.block<3, 3>(3, 3).fullPivLu().solve(imu.getAngularVelocity());

    return false;

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

void XBotInterface::update()
{
    impl->_tmp.setDirty();
    update_impl();
}

void XBotInterface::qToMap(VecConstRef q, JointNameMap& qmap)
{
    check_mat_size(q, getNq(), 1, __func__);

    detail::qToMap(impl->_state, q, qmap);
}

void XBotInterface::vToMap(VecConstRef v, JointNameMap& vmap)
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
    Jrel = Jrel - Jtmp;

    // rotate to base
    Utils::rotate(Jrel, w_T_b.linear().transpose());
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

MatConstRef XBotInterface::computeInertiaInverse() const
{
    throw NotImplemented(__func__);
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

std::map<std::string, ImuSensor::Ptr> XBotInterface::getImu()
{
    return impl->_imu_map;
}

std::map<std::string, ForceTorqueSensor::Ptr> XBotInterface::getForceTorque()
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

        auto [id, iq, iv, nq, nv] = _joint_info[i];

        // assign joint qname and vname
        if(nq == 1)
        {
            _state.qnames[iq] = jname;
        }
        else
        {
            for(int i = 0; i < nq; i++)
            {
                _state.qnames[iq + i] = jname + "__" + std::to_string(i);
            }
        }

        if(nv == 1)
        {
            _state.vnames[iv] = jname;
        }
        else
        {
            for(int i = 0; i < nv; i++)
            {
                _state.vnames[iv + i] = jname + "__" + std::to_string(i);
            }
        }

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
            _state.qmin[iv] = lims ? lims->lower : -M_PI;
            _state.qmax[iv] = lims ? lims->upper : M_PI;
            _state.vmax[iv] = lims ? lims->velocity : infinity;
            _state.taumax[iv] = lims ? lims->effort : infinity;
        }

        if(jptr->type == urdf::Joint::CONTINUOUS)
        {
            _state.qmin[iv] = -M_PI;
            _state.qmax[iv] = M_PI;
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
    J.setZero(6, nv);
    Jarg.setZero(6, nv);
    v.setZero(nv);
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
