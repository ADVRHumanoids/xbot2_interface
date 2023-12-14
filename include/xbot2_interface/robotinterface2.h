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

    virtual bool sense();

    virtual bool move();

    static UniquePtr getRobot(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string robot_type,
                              std::string model_type);

    RobotJoint::Ptr getJoint(string_const_ref name);

    RobotJoint::Ptr getJoint(int i);

    virtual ~RobotInterface();

    friend ReadCmdInterface<RobotInterface>;

    friend WriteCmdInterface<RobotInterface>;

    friend WriteStateInterface<RobotInterface>;

protected:

    class Impl;

    std::unique_ptr<Impl> r_impl;

    RobotInterface(std::unique_ptr<XBotInterface> model);

    virtual bool sense_impl() = 0;

    virtual bool move_impl() = 0;

    // XBotInterface interface
public:

    void update() override;

    int getLinkId(string_const_ref link_name) const override;

    using XBotInterface::getJacobian;
    void getJacobian(int link_id, MatRef J) const override;

    using XBotInterface::getPose;
    Eigen::Affine3d getPose(int link_id) const override;

    void sum(VecConstRef q0, VecConstRef v, Eigen::VectorXd& q1) const override;
    void difference(VecConstRef q1, VecConstRef q0, Eigen::VectorXd& v) const override;

    using XBotInterface::getVelocityTwist;
    Eigen::Vector6d getVelocityTwist(int link_id) const override;

    using XBotInterface::getAccelerationTwist;
    Eigen::Vector6d getAccelerationTwist(int link_id) const override;

    using XBotInterface::getJdotTimesV;
    Eigen::Vector6d getJdotTimesV(int link_id) const override;

    Eigen::Vector3d getCOM() const override;

    void getCOMJacobian(MatRef J) const override;

    double getMass() const override;

    VecConstRef computeInverseDynamics() const override;

protected:

    JointParametrization get_joint_parametrization(string_const_ref jname) override;

};

}

}

#endif // ROBOTINTERFACE2_H
