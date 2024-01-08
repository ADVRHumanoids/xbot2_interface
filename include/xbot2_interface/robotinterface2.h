#ifndef ROBOTINTERFACE2_H
#define ROBOTINTERFACE2_H

#include "xbotinterface2.h"

namespace XBot {

inline namespace v2 {

class XBOT2IFC_API RobotInterface : public XBotInterface,
                        public ReadCmdInterface<RobotInterface>,
                        protected WriteCmdInterface<RobotInterface>,
                        protected WriteStateInterface<RobotInterface>
{

public:

    XBOT_DECLARE_SMART_PTR(RobotInterface);

    virtual bool sense(bool update_model = true);

    virtual bool move();

    static UniquePtr getRobot(ConfigOptions opt);

    static UniquePtr getRobot(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string robot_type,
                              std::string model_type);

    static UniquePtr getRobot(std::string urdf_string,
                              std::string srdf_string,
                              std::string robot_type,
                              std::string model_type);

    static UniquePtr getRobot(std::string urdf_string,
                              std::string robot_type,
                              std::string model_type);

    const ModelInterface& model() const;

    ModelInterface::ConstPtr modelSharedPtr() const;

    RobotJoint::Ptr getJoint(string_const_ref name);

    RobotJoint::Ptr getJoint(int i);

    RobotJoint::ConstPtr getJoint(string_const_ref name) const;

    RobotJoint::ConstPtr getJoint(int i) const;

    const std::vector<RobotJoint::Ptr>& getJoints();

    const std::vector<RobotJoint::ConstPtr>& getJoints() const;

    using ReadCmdInterface::setControlMode;

    void setControlMode(const std::map<std::string, ControlMode::Type>& ctrl_map);

    void setReferenceFrom(const XBotInterface& other, ControlMode::Type mask = ControlMode::ALL);

    virtual ~RobotInterface();

    friend ReadCmdInterface<RobotInterface>;

    friend WriteCmdInterface<RobotInterface>;

    friend WriteStateInterface<RobotInterface>;

protected:

    class Impl;

    std::unique_ptr<Impl> r_impl;

    RobotInterface(std::unique_ptr<ModelInterface> model);

    virtual bool sense_impl() = 0;

    virtual bool move_impl() = 0;

    // XBotInterface interface
public:

    void update_impl() override;

    int getLinkId(string_const_ref link_name) const override;

    using XBotInterface::getJacobian;
    void getJacobian(int link_id, MatRef J) const override;

    using XBotInterface::getPose;
    Eigen::Affine3d getPose(int link_id) const override;

    using XBotInterface::sum;
    void sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const override;

    using XBotInterface::difference;
    void difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const override;

    using XBotInterface::getVelocityTwist;
    Eigen::Vector6d getVelocityTwist(int link_id) const override;

    using XBotInterface::getAccelerationTwist;
    Eigen::Vector6d getAccelerationTwist(int link_id) const override;

    using XBotInterface::getJdotTimesV;
    Eigen::Vector6d getJdotTimesV(int link_id) const override;

    using XBotInterface::getCOM;
    Eigen::Vector3d getCOM() const override;

    using XBotInterface::getCOMJacobian;
    void getCOMJacobian(MatRef J) const override;

    using XBotInterface::getCOMJdotTimesV;
    Eigen::Vector3d getCOMJdotTimesV() const override;

    using XBotInterface::getCOMVelocity;
    Eigen::Vector3d getCOMVelocity() const override;

    using XBotInterface::getCOMAcceleration;
    Eigen::Vector3d getCOMAcceleration() const override;

    using XBotInterface::getMass;
    double getMass() const override;

    using XBotInterface::computeInverseDynamics;
    VecConstRef computeInverseDynamics() const override;

    using XBotInterface::computeGravityCompensation;
    VecConstRef computeGravityCompensation() const override;

    using XBotInterface::computeForwardDynamics;
    VecConstRef computeForwardDynamics() const override;

    using XBotInterface::computeInertiaMatrix;
    MatConstRef computeInertiaMatrix() const override;

    using XBotInterface::computeInertiaInverse;
    MatConstRef computeInertiaInverse() const override;

    using XBotInterface::computeNonlinearTerm;
    VecConstRef computeNonlinearTerm() const override;

    using XBotInterface::computeCentroidalMomentumMatrix;
    MatConstRef computeCentroidalMomentumMatrix() const override;

    using XBotInterface::computeCentroidalMomentum;
    Eigen::Vector6d computeCentroidalMomentum() const override;

protected:

    JointParametrization get_joint_parametrization(string_const_ref jname) override;

};

}

}

#endif // ROBOTINTERFACE2_H
