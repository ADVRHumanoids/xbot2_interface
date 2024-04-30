#include "common.h"


using TestRobot = TestWithModel;

TEST_F(TestRobot, checkCtrl)
{
    auto ctrl = robot->getControlMode();

    ASSERT_EQ(ctrl.size(), robot->getJointNum());

    ASSERT_TRUE(ctrl.isConstant(255));

    robot->setControlMode(XBot::ControlMode::None());

    ASSERT_TRUE(ctrl.isConstant(0));

    robot->setControlMode(XBot::ControlMode::ALL);

    ASSERT_TRUE(ctrl.isConstant(XBot::ControlMode::ALL));

    auto mask = robot->getValidCommandMask();

    ASSERT_TRUE(mask.isConstant(0));

    robot->getJoint(1)->setStiffness(Eigen::Scalard(1.0));

    ASSERT_EQ(mask[1], XBot::ControlMode::STIFFNESS);

    ASSERT_EQ(robot->getJoint(1)->getValidCommandMask().value(), XBot::ControlMode::STIFFNESS);

    XBot::ControlMode ctrlmode = static_cast<XBot::ControlMode::Type>(robot->getJoint(1)->getValidCommandMask().value());

    EXPECT_TRUE(ctrlmode.isStiffnessEnabled());

}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
