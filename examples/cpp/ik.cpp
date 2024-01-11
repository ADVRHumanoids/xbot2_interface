#include "common.h"
#include <xbot2_interface/common/utils.h>

int main(int argc, char **argv)
{
    auto urdf_file = resources.open("resources/mobile_manipulator_3dof.urdf");
    std::string urdf(urdf_file.begin(), urdf_file.end());

    auto model = XBot::ModelInterface::getModel(urdf, "pin");

    std::string ee_name = "arm_link3";

    // set model to random configuration
    model->setJointPosition(model->generateRandomQ());
    model->update();

    // target pose
    Eigen::Affine3d Tdes;
    Tdes.setIdentity();

    // ik loop
    for(int i = 0; true; i++)
    {
        // evaluate fk
        Eigen::Affine3d T = model->getPose(ee_name);

        // evaluate pose error
        Eigen::Vector6d err = XBot::Utils::computePoseError(Tdes, T);

        if(err.lpNorm<Eigen::Infinity>() < 1e-6)
        {
            std::cout << "solved \n\n";
            break;
        }

        std::cout << "iter " << i << "\terr = " << err.transpose().format(2) << "\n\n";

        // evaluate jacobian
        Eigen::MatrixXd J = model->getJacobian(ee_name);

        // compute step
        Eigen::VectorXd dq = J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(err);

        // integrate model with step
        model->integrateJointPosition(dq);

        model->update();

    }

    std::cout << "final pose: \n" << model->getPose(ee_name).matrix() << std::endl;

}
