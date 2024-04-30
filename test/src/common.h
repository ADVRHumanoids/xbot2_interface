#ifndef COMMON_H
#define COMMON_H




#include <chrono>

#include <gtest/gtest.h>

#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/common/utils.h>

std::string model_type = "pin",
robot_type = "ros",
urdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.urdf",
srdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.srdf";

#define TIC(name) auto tic_##name = std::chrono::high_resolution_clock::now()
#define TOC(name) std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - tic_##name).count()



class TestCommon : public testing::Test
{

protected:

    urdf::ModelSharedPtr urdf;
    srdf::ModelSharedPtr srdf;

    TestCommon(){}

    virtual ~TestCommon() {
    }

    virtual void SetUp()
    {
        urdf = std::make_shared<urdf::Model>();
        ASSERT_TRUE(urdf->initFile(urdf_path));

        if(!srdf_path.empty())
        {
            srdf = std::make_shared<srdf::Model>();
            ASSERT_TRUE(srdf->initFile(*urdf, srdf_path));
        }
    }

    virtual void TearDown() {
    }

};

namespace XBot {

class RobotInterfaceMockup : public RobotInterface
{

public:

    RobotInterfaceMockup(std::unique_ptr<ModelInterface> model):
        RobotInterface(std::move(model))
    {

    }

    // RobotInterface interface
protected:
    bool sense_impl() override;
    bool move_impl() override;
    bool validateControlMode(string_const_ref jname, ControlMode::Type ctrl) override;
};

bool RobotInterfaceMockup::sense_impl()
{
    return true;
}

bool RobotInterfaceMockup::move_impl()
{
    return true;
}

bool RobotInterfaceMockup::validateControlMode(string_const_ref jname, ControlMode::Type ctrl)
{
    return true;
}

}

class TestWithModel : public TestCommon
{

protected:

    XBot::ModelInterface::Ptr model;

    XBot::RobotInterfaceMockup::Ptr robot;

    void checkConfigurationIsEqual(const XBot::ModelInterface& m1,
                                   const XBot::ModelInterface& m2)
    {
        for(auto [lname, lptr] : m1.getUrdf()->links_)
        {
            auto T1 = m1.getPose(lname);
            auto T2 = m2.getPose(lname);
            EXPECT_TRUE(T1.isApprox(T2, 1e-4)) <<
                lname << "\n" << T1.matrix() << "\n" << T2.matrix() << "\n";
        }
    }

    virtual void SetUp()
    {
        TestCommon::SetUp();

        model = XBot::ModelInterface::getModel(urdf, srdf, model_type);

        robot = std::make_shared<XBot::RobotInterfaceMockup>(XBot::ModelInterface::getModel(urdf, srdf, model_type));
    }

};

#endif // COMMON_H
