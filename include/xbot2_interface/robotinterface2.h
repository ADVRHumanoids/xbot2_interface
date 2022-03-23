#ifndef ROBOTINTERFACE2_H
#define ROBOTINTERFACE2_H

#include "xbotinterface2.h"

namespace XBot {

class RobotInterface2 : public XBotInterface2,
                        public ReadCmdInterface<RobotInterface2>,
                        protected WriteCmdInterface<RobotInterface2>,
                        protected WriteStateInterface<RobotInterface2>
{

public:

    XBOT_DECLARE_SMART_PTR(RobotInterface2);

    virtual bool sense() = 0;

    virtual bool move() = 0;

    static UniquePtr getRobot(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string robot_type,
                              std::string model_type);

    RobotJoint::Ptr getJoint(string_const_ref name);

    RobotJoint::Ptr getJoint(int i);

    virtual ~RobotInterface2();

    friend ReadCmdInterface<RobotInterface2>;
    friend WriteCmdInterface<RobotInterface2>;
    friend WriteStateInterface<RobotInterface2>;

protected:

    class Impl;

    std::unique_ptr<Impl> r_impl;

    RobotInterface2(std::unique_ptr<XBotInterface2> model);

    // XBotInterface2 interface
public:
    void update() override;
    MatConstRef getJacobian(string_const_ref link_name) const override;
    Eigen::Affine3d getPose(string_const_ref link_name) const override;
    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;
    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;
    Eigen::Vector6d getVelocityTwist(string_const_ref link_name) const override;
    VecConstRef computeInverseDynamics() const override;

protected:

    JointParametrization get_joint_parametrization(string_const_ref jname) override;

};

}

#endif // ROBOTINTERFACE2_H
