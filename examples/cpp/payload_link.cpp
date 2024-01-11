#include "common.h"

int main(int argc, char **argv)
{
    auto urdf_file = resources.open("resources/mobile_manipulator_3dof.urdf");
    std::string urdf(urdf_file.begin(), urdf_file.end());

    auto model = XBot::ModelInterface::getModel(urdf, "pin");

    // set random q
    model->setJointPosition(model->generateRandomQ());
    model->update();

    // compute gcomp
    std::cout << "gcomp without payload: \n" << model->computeGravityCompensation().transpose().format(3) << "\n\n";

    // add a zero mass payload link
    int payload_id = model->addFixedLink("payload", "arm_link3", 0.0, Eigen::Matrix3d::Zero(), Eigen::Affine3d::Identity());
    model->update();

    // compute gcomp
    std::cout << "gcomp with zero payload: \n" << model->computeGravityCompensation().transpose().format(3) << "\n\n";

    // add mass
    Eigen::Affine3d ee_T_payload;
    ee_T_payload.setIdentity();
    ee_T_payload.translation().setConstant(0.1);
    model->updateFixedLink(payload_id, 10.0, Eigen::Matrix3d::Identity(), ee_T_payload);
    model->update();
    std::cout << "gcomp with 10 kg payload: \n" << model->computeGravityCompensation().transpose().format(3) << "\n\n";

    // payload jacobian
    Eigen::MatrixXd Jpay(6, model->getNv());
    model->getJacobian(payload_id, Jpay);

    // payload pose
    auto w_T_payload = model->getPose(payload_id);

}
