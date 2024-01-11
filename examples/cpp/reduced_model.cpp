#include "common.h"

int main(int argc, char **argv)
{
    auto urdf_file = resources.open("resources/mobile_manipulator_3dof.urdf");
    std::string urdf(urdf_file.begin(), urdf_file.end());

    auto model = XBot::ModelInterface::getModel(urdf, "pin");

    auto qrand = model->generateRandomQ();

    auto redmodel = model->generateReducedModel(qrand, {"world_to_base", "base_to_wheel"});

    std::cout << "*** original model *** \n";
    model->print(std::cout) << "\n\n";

    std::cout << "*** reduced model *** \n";
    redmodel->print(std::cout) << "\n\n";

    model->setJointPosition(qrand);
    model->update();

    redmodel->syncFrom(*model);
    redmodel->update();

    std::cout << "*** original model fk *** \n" << model->getPose("arm_link3").matrix() << "\n\n";
    std::cout << "*** reduced  model fk *** \n" << redmodel->getPose("arm_link3").matrix() << "\n";
}
