#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include "types.h"

namespace urdf {
class ModelInterface;
}

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

XBOT2IFC_API const Eigen::Vector6d& rotate(Eigen::Vector6d& vel,
                              const Eigen::Matrix3d& R);

template <typename Mat, typename MatOutput>
const Eigen::MatrixBase<MatOutput>& rotate(const Eigen::MatrixBase<Mat>& J,
                                     const Eigen::Matrix3d& R,
                                     Eigen::MatrixBase<MatOutput>& output)
{
    output.resize(J.rows(), J.cols());
    output.template topRows<3>().noalias() = R * J.template topRows<3>();
    output.template bottomRows<3>().noalias() = R * J.template bottomRows<3>();
    return output;
}

XBOT2IFC_API std::string urdfToString(const urdf::ModelInterface& urdf);

}

}

#endif // UTILS_H
