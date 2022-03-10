#include "src/xbotinterface2.h"

int main()
{
    std::string urdf_path = "/opt/xbot/share/centauro_urdf/urdf/centauro.urdf";
    std::string srdf_path = "/opt/xbot/share/centauro_srdf/srdf/centauro.srdf";

    auto urdf = std::make_shared<urdf::Model>();
    urdf->initFile(urdf_path);

    auto srdf = std::make_shared<srdf::Model>();
    srdf->initFile(*urdf, srdf_path);

    auto model = XBot::XBotInterface2::getModel(urdf, srdf, "pin");

    std::cout << model->getJointPosition().transpose() << std::endl;

    std::cout << model->getRobotState("home").transpose() << std::endl;

    model->setJointPosition(model->getRobotState("home"));

    model->update();

    std::cout << model->getJointPosition().transpose() << std::endl;

    std::cout << model->getPose("pelvis").matrix() << "\n\n";

    std::cout << model->getPose("arm1_7").matrix() << "\n\n";

    std::cout << model->getPose("contact_1").matrix() << "\n\n";

    std::cout << model->getJacobian("contact_1") << "\n\n";

    std::cout << model->getJoint("reference")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_wheel_2")->getJointPosition().transpose() << "\n\n";

    std::cout << model->getJoint("j_arm1_4")->getJointPosition().transpose() << "\n\n";
}
