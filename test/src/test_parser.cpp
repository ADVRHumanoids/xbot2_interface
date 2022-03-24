#include <gtest/gtest.h>

#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/robotinterface2.h>

std::string model_type = "pin",
robot_type = "ros",
urdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.urdf",
srdf_path = XBOT2_TEST_RESOURCE_DIR "centauro_capsule.srdf";

class TestParser : public testing::Test
{

protected:

    urdf::ModelSharedPtr urdf;
    srdf::ModelSharedPtr srdf;

    TestParser(){}

    virtual ~TestParser() {
    }

    virtual void SetUp()
    {
        urdf = std::make_shared<urdf::Model>();
        urdf->initFile(urdf_path);

        if(!srdf_path.empty())
        {
            srdf = std::make_shared<srdf::Model>();
            srdf->initFile(*urdf, srdf_path);
        }
    }

    virtual void TearDown() {
    }


};

TEST_F(TestParser, testUrdfOnly)
{
    auto model = XBot::ModelInterface2::getModel(urdf,
                                                 srdf,
                                                 model_type);

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
