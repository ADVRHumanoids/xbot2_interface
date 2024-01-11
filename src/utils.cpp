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

std::string XBot::Utils::srdfToString(const urdf::ModelInterface &urdf, const srdf::Model &srdf)
{
    srdf::SRDFWriter writer;
    writer.initModel(urdf, srdf);
    return writer.getSRDFString();
}

void XBot::Utils::computeOrientationError(const Eigen::Matrix3d &ref,
                             const Eigen::Matrix3d &actual,
                             Eigen::Vector3d &error)
{

    Eigen::Quaterniond q(actual), q_d(ref);

    if(q.dot(q_d) < 0){
        q.x() *= -1;
        q.y() *= -1;
        q.z() *= -1;
        q.w() *= -1;
    }

    error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());

}

Eigen::Vector3d XBot::Utils::computeOrientationError(const Eigen::Matrix3d &ref,
                                                     const Eigen::Matrix3d &actual)
{
    Eigen::Vector3d ret;
    computeOrientationError(ref, actual, ret);
    return ret;
}

Eigen::Matrix6d XBot::Utils::adjointFromRotation(const Eigen::Matrix3d &R) {

    Eigen::Matrix6d I;
    I.setZero();

    I.block<3, 3>(0, 0) = R;
    I.block<3, 3>(3, 3) = R;

    return I;
}

double XBot::Utils::v2::quinticSpline(double tau)
{
    // quintic poly 6t^5 - 15t^4 + 10t^3
    if(tau < 0) return 0;
    if(tau > 1) return 1;
    return ((6*tau - 15)*tau + 10)*tau*tau*tau;
}

double XBot::Utils::v2::quinticSpline(double t0, double tf, double time)
{
    double tau = (time - t0)/(tf - t0);
    return quinticSpline(tau);
}

std::tuple<double, double, double> XBot::Utils::v2::quinticSplineDerivatives(double tau)
{
    if(tau < 0) return {0, 0, 0};
    if(tau > 1) return {1, 0, 0};
    return {
        quinticSpline(tau),
        30*(-1 + tau)*(-1 + tau)*tau*tau,
        60*tau*(1 - 3*tau + 2*tau*tau)
    };
}

std::tuple<double, double, double> XBot::Utils::v2::quinticSplineDerivatives(double t0,
                                                                             double tf,
                                                                             double time)
{
    double dt = tf - t0;
    double tau = (time - t0)/dt;
    auto [y, dy, ddy] =  quinticSplineDerivatives(tau);
    dy /= dt;
    ddy /= (dt*dt);
    return {y, dy, ddy};
}

XBot::ControlMode::Type XBot::operator|(ControlMode::Type a, ControlMode::Type b)
{
    return static_cast<ControlMode::Type>(static_cast<int>(a) | static_cast<int>(b));
}

XBot::ControlMode::Type XBot::operator&(ControlMode::Type a, ControlMode::Type b)
{
    return static_cast<ControlMode::Type>(static_cast<int>(a) & static_cast<int>(b));
}

Eigen::Vector6d XBot::Utils::computePoseError(const Eigen::Affine3d &ref, const Eigen::Affine3d &actual)
{
    Eigen::Vector6d ret;
    ret << ref.translation() - actual.translation(),
        computeOrientationError(ref.linear(), actual.linear());
    return ret;
}
