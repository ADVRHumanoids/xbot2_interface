#include <xbot2_interface/common/utils.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <sstream>

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
    return R.eulerAngles(0, 1, 2);
}

Eigen::Matrix3d XBot::Utils::skew(const Eigen::Vector3d &r)
{
    Eigen::Matrix3d S;
    S <<        0, -r[2],  r[1],
             r[2],     0, -r[0],
            -r[1],  r[0],     0;
    return S;
}

const Eigen::Vector6d& XBot::Utils::rotate(Eigen::Vector6d& vel,
                                           const Eigen::Matrix3d& R)
{
    vel.topRows<3>() = R * vel.topRows<3>();
    vel.bottomRows<3>() = R * vel.bottomRows<3>();
    return vel;
}


std::string XBot::Utils::urdfToString(const urdf::ModelInterface &urdf)
{
    auto tiXmlDoc(urdf::exportURDF(urdf));
    TiXmlPrinter printer;
    tiXmlDoc->Accept(&printer);
    return printer.Str();
}


