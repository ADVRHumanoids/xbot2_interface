#ifndef XBOTINTERFACE2_H
#define XBOTINTERFACE2_H

#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include "joint.h"
#include "imu.h"
#include "force_torque.h"

namespace XBot {

inline namespace v2 {

class RobotInterface;

class ModelInterface;


class XBOT2IFC_API XBotInterface : public ReadStateInterface<XBotInterface>
{

public:

    XBOT_DECLARE_SMART_PTR(XBotInterface);

    struct ConfigOptions
    {
        urdf::ModelConstSharedPtr urdf;
        srdf::ModelConstSharedPtr srdf;

        bool set_urdf(std::string urdf_string);

        bool set_srdf(std::string srdf_string);

    };

    explicit XBotInterface(const ConfigOptions& opt);

    XBotInterface(urdf::ModelConstSharedPtr urdf,
                  srdf::ModelConstSharedPtr srdf = nullptr);

    XBotInterface(XBotInterface&&) = default;

    const std::string& getName() const;

    urdf::ModelConstSharedPtr getUrdf() const;

    srdf::ModelConstSharedPtr getSrdf() const;

    ConfigOptions getConfigOptions() const;

    bool hasRobotState(string_const_ref name) const;

    Eigen::VectorXd getRobotState(string_const_ref name) const;

    bool getRobotState(string_const_ref name, Eigen::VectorXd& q) const;

    int getJointNum() const;

    bool hasJoint(string_const_ref name) const;

    Joint::ConstPtr getJoint(string_const_ref joint_name) const;

    Joint::ConstPtr getJoint(int id) const;

    const JointInfo& getJointInfo(string_const_ref joint_name) const;

    const JointInfo& getJointInfo(int id) const;

    int getJointId(string_const_ref joint_name) const;

    int getDofIndex(string_const_ref joint_name) const;

    const std::vector<std::string>& getJointNames() const;

    const std::vector<Joint::Ptr>& getJoints();

    const std::vector<Joint::ConstPtr>& getJoints() const;

    void update();

    virtual int getLinkId(string_const_ref link_name) const = 0;

    /* Eigen/map conversions */

    void qToMap(VecConstRef q, JointNameMap& qmap);

    void vToMap(VecConstRef v, JointNameMap& vmap);

    void mapToQ(const JointNameMap& qmap, Eigen::VectorXd& q) const;

    void mapToV(const JointNameMap& vmap, Eigen::VectorXd& v) const;

    /* Joint limits */

    bool checkJointLimits(VecConstRef q) const;

    VecRef enforceJointLimits(Eigen::VectorXd& q, double tol = 0.0) const;

    void generateRandomQ(Eigen::VectorXd& qrand) const;

    Eigen::VectorXd generateRandomQ() const;


    /* Attached sensors */

    std::map<std::string, ImuSensor::ConstPtr> getImu() const;

    ImuSensor::ConstPtr getImu(string_const_ref name) const;

    std::map<std::string, ForceTorqueSensor::ConstPtr> getForceTorque() const;

    ForceTorqueSensor::ConstPtr getForceTorque(string_const_ref name) const;

    bool addSensor(Sensor::Ptr s);


    /* Floating base */

    bool isFloatingBase() const;

    bool getFloatingBaseLink(std::string& fb) const;

    string_const_ref getFloatingBaseLink() const;


    /* Jacobians */

    // absolute
    virtual void getJacobian(int link_id, MatRef J) const = 0;

    bool getJacobian(string_const_ref link_name, MatRef J) const;

    bool getJacobian(string_const_ref link_name, Eigen::MatrixXd& J) const;

    Eigen::MatrixXd getJacobian(string_const_ref link_name) const;

    // relative
    virtual void getRelativeJacobian(int distal_id, int base_id, MatRef J) const;

    bool getRelativeJacobian(string_const_ref distal_name, string_const_ref base_name, MatRef J) const;

    bool getRelativeJacobian(string_const_ref distal_name, string_const_ref base_name, Eigen::MatrixXd& J) const;

    Eigen::MatrixXd getRelativeJacobian(string_const_ref distal_name, string_const_ref base_name) const;

    /* Forward kinematics */

    // pose (absolute)
    virtual Eigen::Affine3d getPose(int link_id) const = 0;

    Eigen::Affine3d getPose(string_const_ref link_name) const;

    bool getPose(string_const_ref link_name, Eigen::Affine3d& w_T_l) const;

    // pose (relative)
    Eigen::Affine3d getPose(int distal_id, int base_id) const;

    Eigen::Affine3d getPose(string_const_ref distal_name, string_const_ref base_name) const;

    bool getPose(string_const_ref distal_name, string_const_ref base_name, Eigen::Affine3d& w_T_l) const;

    // velocity twist (absolute)
    virtual Eigen::Vector6d getVelocityTwist(int link_id) const;

    Eigen::Vector6d getVelocityTwist(string_const_ref link_name) const;

    bool getVelocityTwist(string_const_ref link_name, Eigen::Vector6d& v) const;

    // acceleration twist (absolute)
    virtual Eigen::Vector6d getAccelerationTwist(int link_id) const;

    Eigen::Vector6d getAccelerationTwist(string_const_ref link_name) const;

    bool getAccelerationTwist(string_const_ref link_name, Eigen::Vector6d& a) const;

    //
    virtual Eigen::Vector6d getJdotTimesV(int link_id) const;

    Eigen::Vector6d getJdotTimesV(string_const_ref link_name) const;

    bool getJdotTimesV(string_const_ref link_name, Eigen::Vector6d& a) const;

    // velocity twist (relative)
    Eigen::Vector6d getRelativeVelocityTwist(int distal_id, int base_id) const;

    Eigen::Vector6d getRelativeVelocityTwist(string_const_ref distal_name, string_const_ref base_name) const;

    bool getRelativeVelocityTwist(string_const_ref distal_name, string_const_ref base_name, Eigen::Vector6d& v) const;

    // acceleration twist (relative)
    Eigen::Vector6d getRelativeAccelerationTwist(int distal_id, int base_id) const;

    Eigen::Vector6d getRelativeAccelerationTwist(string_const_ref distal_name, string_const_ref base_name) const;

    bool getRelativeAccelerationTwist(string_const_ref distal_name, string_const_ref base_name, Eigen::Vector6d& v) const;

    // com

    virtual Eigen::Vector3d getCOM() const = 0;

    virtual void getCOMJacobian(MatRef J) const = 0;

    bool getCOMJacobian(Eigen::MatrixXd& J) const;

    Eigen::MatrixXd getCOMJacobian() const;


    //
    Eigen::Vector6d getRelativeJdotTimesV(int distal_id, int base_id) const;

    Eigen::Vector6d getRelativeJdotTimesV(string_const_ref distal_name, string_const_ref base_name) const;

    bool getRelativeJdotTimesV(string_const_ref distal_name, string_const_ref base_name, Eigen::Vector6d& v) const;

    // dynamics
    virtual double getMass() const = 0;

    virtual VecConstRef computeInverseDynamics() const = 0;

    virtual VecConstRef computeGravityCompensation() const = 0;

    void computeInverseDynamics(Eigen::VectorXd& rnea) const;

    void computeGravityCompensation(Eigen::VectorXd& gcomp) const;

    virtual VecConstRef computeForwardDynamics() const = 0;

    virtual MatConstRef computeInertiaMatrix() const = 0;

    virtual MatConstRef computeInertiaInverse() const;

    // manifold operations
    virtual void sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const = 0;

    virtual void difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const = 0;

    Eigen::VectorXd sum(VecConstRef q0, VecConstRef v) const;

    Eigen::VectorXd difference(VecConstRef q1, VecConstRef q0) const;

    //
    virtual ~XBotInterface();

    friend ReadStateInterface<XBotInterface>;

    friend WriteStateInterface<ModelInterface>;

    friend RobotInterface;

protected:

    virtual void update_impl() = 0;

    void finalize();

    struct JointParametrization
    {
        JointInfo info;

        Eigen::VectorXd q0;

        std::function<void(VecConstRef, VecRef)> fn_minimal_to_q;

        std::function<void(VecConstRef, VecRef)> fn_q_to_minimal;

        std::function<void(VecConstRef,
                           VecConstRef,
                           Eigen::Affine3d*,
                           Eigen::Vector6d*)> fn_fwd_kin;

        std::function<void(const Eigen::Affine3d& p_T_c,
                           const Eigen::Vector6d& c_vc,
                           VecRef q,
                           VecRef v)> fn_inv_kin;

    };


    virtual JointParametrization get_joint_parametrization(string_const_ref jname);

    std::map<std::string, ImuSensor::Ptr> getImu();

    std::map<std::string, ForceTorqueSensor::Ptr> getForceTorque();


protected:

    UniversalJoint::Ptr getUniversalJoint(string_const_ref name);

    UniversalJoint::Ptr getUniversalJoint(int i);

    class Impl;

    XBotInterface(std::shared_ptr<Impl> impl);

    std::shared_ptr<Impl> impl;

};


class XBOT2IFC_API ModelInterface : public XBotInterface,
        public WriteStateInterface<ModelInterface>
{

public:

    XBOT_DECLARE_SMART_PTR(ModelInterface);

    using XBotInterface::XBotInterface;

    static UniquePtr getModel(std::string urdf_string,
                              std::string type);

    static UniquePtr getModel(std::string urdf_string,
                              std::string srdf_string,
                              std::string type);

    static UniquePtr getModel(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string type);

    void syncFrom(const XBotInterface& other);

    std::string getType() const;

    virtual UniquePtr clone() = 0;

    UniquePtr generateReducedModel(VecConstRef q, std::vector<std::string> joints_to_fix) const;

    ModelJoint::Ptr getJoint(string_const_ref name);

    ModelJoint::Ptr getJoint(int i);

    ModelJoint::ConstPtr getJoint(string_const_ref name) const;

    ModelJoint::ConstPtr getJoint(int i) const;

    const std::vector<ModelJoint::Ptr>& getJoints();

    const std::vector<ModelJoint::ConstPtr>& getJoints() const;

    /* Floating base */

    bool setFloatingBaseState(const Eigen::Affine3d& w_T_b, const Eigen::Vector6d& v);

    bool setFloatingBaseState(const ImuSensor& imu);

    bool setFloatingBasePose(const Eigen::Affine3d& w_T_b);

    bool setFloatingBaseTwist(const Eigen::Vector6d& v);

    virtual ~ModelInterface();

};

using ConfigOptions = XBotInterface::ConfigOptions;

}

}

#endif // XBOTINTERFACE2_H
