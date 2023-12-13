#include <xbot2_interface/xbotinterface2.h>
#include <xbot2_interface/robotinterface2.h>

using namespace XBot;

int main(int argc, char **argv)
{
    std::string urdf_path = "/opt/xbot/share/centauro_urdf/urdf/centauro.urdf";
    std::string srdf_path = "/opt/xbot/share/centauro_srdf/srdf/centauro.srdf";

    auto urdf = std::make_shared<urdf::Model>();
    urdf->initFile(urdf_path);

    auto srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);

    auto model = XBot::ModelInterface::getModel(urdf, srdf, argv[1]);

    std::cout << model->getJointPosition().transpose() << "\n\n";

    std::cout << model->getRobotState("home").transpose() << "\n\n";

    model->setJointPosition(model->getRobotState("home"));

    model->update();

    std::cout << model->getJointPosition().transpose() << "\n\n";

    std::cout << model->getPose("pelvis").matrix() << "\n\n";

    std::cout << model->getPose("arm1_7").matrix() << "\n\n";

    std::cout << model->getPose("contact_1").matrix() << "\n\n";

    Eigen::MatrixXd J;
    model->getJacobian("contact_1", J);
    std::cout << J.leftCols(12) << "\n\n";

    std::cout << model->getJoint("reference")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_wheel_2")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_arm1_4")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_arm1_4")->getJointPositionMinimal() << "\n\n";

    auto robot = XBot::RobotInterface::getRobot(urdf, srdf, "ros", argv[1]);

    robot->getJoint("j_wheel_2")->setPositionReferenceMinimal(from_value(1.0));


    while(true)
    {
        if(!robot->sense())
        {
            continue;
        }

        robot->update();

        std::cout << "pos " << robot->getJointPosition().transpose() << "\n\n";

        std::cout << "vel " << robot->getJointVelocity().transpose() << "\n\n";

        std::cout << "ee " << robot->getPose("arm1_8").translation().transpose() << "\n\n";

        robot->getJoint("j_wheel_2")->setPositionReferenceMinimal(from_value(1.0));

        robot->getJoint(0)->setVelocityReference(Eigen::Vector6d::Ones());

        robot->move();

        usleep(10000);

    }



}
