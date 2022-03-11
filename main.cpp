#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/robotinterface2.h>


int main(int argc, char **argv)
{
    std::string urdf_path = "/opt/xbot/share/centauro_urdf/urdf/centauro.urdf";
    std::string srdf_path = "/opt/xbot/share/centauro_srdf/srdf/centauro.srdf";

    auto urdf = std::make_shared<urdf::Model>();
    urdf->initFile(urdf_path);

    auto srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);

    auto model = XBot::XBotInterface2::getModel(urdf, srdf, argv[1]);

    std::cout << model->getJointPosition().transpose() << "\n\n";

    std::cout << model->getRobotState("home").transpose() << "\n\n";

    model->setJointPosition(model->getRobotState("home"));

    model->update();

    std::cout << model->getJointPosition().transpose() << "\n\n";

    std::cout << model->getPose("pelvis").matrix() << "\n\n";

    std::cout << model->getPose("arm1_7").matrix() << "\n\n";

    std::cout << model->getPose("contact_1").matrix() << "\n\n";

    std::cout << model->getJacobian("contact_1").leftCols(12) << "\n\n";

    std::cout << model->getJoint("reference")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_wheel_2")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_arm1_4")->getJointPosition().transpose() << "\n\n";

    auto robot = XBot::RobotInterface2::getRobot(urdf, srdf, "ros", argv[1]);

    while(true)
    {
        if(!robot->sense())
        {
            continue;
        }

        std::cout << robot->getJointPosition().transpose() << "\n\n";

        std::cout << robot->getJointVelocity().transpose() << "\n\n";

        std::cout << robot->getJointEffort().transpose() << "\n\n";

        usleep(10000);

    }



}
