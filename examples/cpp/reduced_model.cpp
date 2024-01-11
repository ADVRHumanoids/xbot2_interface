#include "common.h"

int main(int argc, char **argv)
{
    auto urdf_file = resources.open("resources/mobile_manipulator_3dof.urdf");
    std::string urdf(urdf_file.begin(), urdf_file.end());

    // load model from urdf
    auto model = XBot::ModelInterface::getModel(urdf, "pin");

    // generate a random q
    auto qrand = model->generateRandomQ();

    // reduce model by fixing the base floating joint and the wheel joint
    auto redmodel = model->generateReducedModel(qrand, {"world_to_base", "base_to_wheel"});

    // print models
    std::cout << "*** original model *** \n";
    model->print(std::cout) << "\n\n";

    std::cout << "*** reduced model *** \n";
    redmodel->print(std::cout) << "\n\n";

    // set the random q to the original model
    model->setJointPosition(qrand);
    model->update();

    // sync the reduced model with the original one
    // (this takes care of the different structure between the two models)
    redmodel->syncFrom(*model);
    redmodel->update();

    // check that the end effector pose is the same
    std::cout << "*** original model fk *** \n" << model->getPose("arm_link3").matrix() << "\n\n";
    std::cout << "*** reduced  model fk *** \n" << redmodel->getPose("arm_link3").matrix() << "\n";
}
