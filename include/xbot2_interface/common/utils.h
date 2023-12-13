#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include "types.h"

namespace XBot::Utils {

inline namespace v2 {

XBOT2IFC_API Eigen::Matrix3d rpyToRotationMatrix(Eigen::Ref<const Eigen::Vector3d> rpy);

XBOT2IFC_API Eigen::Matrix3d rpyJacobian(Eigen::Ref<const Eigen::Vector3d> rpy);

XBOT2IFC_API Eigen::Vector3d rotationMatrixToRpy(const Eigen::Matrix3d& R);

XBOT2IFC_API Eigen::Matrix3d skew(const Eigen::Vector3d& r);

template <typename Mat>
void changeRefPoint(Eigen::MatrixBase<Mat>& J_or_vel,
                    const Eigen::Vector3d& p)
{
    // v(P) = v(0) + om x (P - O)
    Eigen::Matrix3d S = skew(p);
    J_or_vel.template topRows<3>().noalias() -= S * J_or_vel.template bottomRows<3>();
}

template <typename Mat>
const Eigen::MatrixBase<Mat>& rotate(Eigen::MatrixBase<Mat>& J_or_vel,
                                     const Eigen::Matrix3d& R)
{
    J_or_vel.template topRows<3>() = R * J_or_vel.template topRows<3>();
    J_or_vel.template bottomRows<3>() = R * J_or_vel.template bottomRows<3>();
    return J_or_vel;
}

}

}

#endif // UTILS_H
