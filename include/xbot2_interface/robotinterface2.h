#ifndef ROBOTINTERFACE2_H
#define ROBOTINTERFACE2_H

#include "xbotinterface2.h"

namespace XBot {

class RobotInterface2 : public XBotInterface2,
                        public ReadCmdInterface<RobotInterface2>,
                        protected WriteCmdInterface<RobotInterface2>
{

public:

    XBOT_DECLARE_SMART_PTR(RobotInterface2);

    virtual bool sense() = 0;

    virtual bool move() = 0;

    static UniquePtr getRobot(urdf::ModelConstSharedPtr urdf,
                              srdf::ModelConstSharedPtr srdf,
                              std::string robot_type,
                              std::string model_type);

    ~RobotInterface2();

    friend ReadCmdInterface<RobotInterface2>;
    friend WriteCmdInterface<RobotInterface2>;

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

    RobotInterface2(std::unique_ptr<XBotInterface2> model);

    // hide unwanted
    using WriteStateInterface<XBotInterface2>::setJointPosition;
    using WriteStateInterface<XBotInterface2>::setJointVelocity;
    using WriteStateInterface<XBotInterface2>::setJointAcceleration;
    using WriteStateInterface<XBotInterface2>::setJointEffort;


    // XBotInterface2 interface
public:
    void update() override;
    MatConstRef getJacobian(std::string_view link_name) const override;
    Eigen::Affine3d getPose(std::string_view link_name) const override;
    VecConstRef sum(VecConstRef q0, VecConstRef v) const override;
    VecConstRef difference(VecConstRef q1, VecConstRef q0) const override;

protected:
    JointParametrization get_joint_parametrization(std::string_view jname) override;
};

}

#endif // ROBOTINTERFACE2_H
