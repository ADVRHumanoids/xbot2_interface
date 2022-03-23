#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>

namespace XBot::Utils {

Eigen::Matrix3d rpyToRotationMatrix(Eigen::Ref<const Eigen::Vector3d> rpy);

Eigen::Matrix3d rpyJacobian(Eigen::Ref<const Eigen::Vector3d> rpy);

Eigen::Vector3d rotationMatrixToRpy(const Eigen::Matrix3d& R);

}

#endif // UTILS_H
