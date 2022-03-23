#include <xbot2_interface/common/utils.h>


Eigen::Matrix3d XBot::Utils::rpyToRotationMatrix(Eigen::Ref<const Eigen::Vector3d> rpy)
{
    Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());

    return Eigen::Matrix3d { rollAngle * pitchAngle * yawAngle };
}

Eigen::Matrix3d XBot::Utils::rpyJacobian(Eigen::Ref<const Eigen::Vector3d> rpy)
{
    Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());

    Eigen::Matrix3d om_J_v;
    om_J_v.col(0) = Eigen::Vector3d::UnitX();
    om_J_v.col(1) = rollAngle * Eigen::Vector3d::UnitY();
    om_J_v.col(2) = rollAngle * (pitchAngle * Eigen::Vector3d::UnitZ());

    return om_J_v;
}

Eigen::Vector3d XBot::Utils::rotationMatrixToRpy(const Eigen::Matrix3d &R)
{

}
