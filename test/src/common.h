#ifndef COMMON_H
#define COMMON_H

#include <gtest/gtest.h>

#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/robotinterface2.h>

std::string model_type = "pin",
robot_type = "ros",
urdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.urdf",
srdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.srdf";

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

class TestWithModel : public TestCommon
{

protected:

    XBot::ModelInterface2::Ptr model;

    virtual void SetUp()
    {
        TestCommon::SetUp();

        model = XBot::ModelInterface2::getModel(urdf, srdf, model_type);
    }

};

#endif // COMMON_H
