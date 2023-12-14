#include "impl/imu.hxx"

using namespace XBot;

ImuSensor::Impl::Impl()
{
    omega.setZero();
    acc.setZero();
    rot.setIdentity();
}

ImuSensor::ImuSensor(string_const_ref name):
    Sensor(name)
{
    impl = std::make_unique<Impl>();
}

Eigen::Vector3d ImuSensor::getAngularVelocity() const
{
    return impl->omega;
}

Eigen::Vector3d ImuSensor::getLinearAcceleration() const
{
    return impl->acc;
}

Eigen::Quaterniond ImuSensor::getOrientation() const
{
    return impl->rot;
}

void ImuSensor::getAngularVelocity(Eigen::Vector3d &omega) const
{
    omega = impl->omega;
}

void ImuSensor::getLinearAcceleration(Eigen::Vector3d &acc) const
{
    acc = impl->acc;
}

void ImuSensor::getOrientation(Eigen::Quaterniond &rot) const
{
    rot = impl->rot;
}

void ImuSensor::setMeasurement(const Eigen::Vector3d &omega,
                               const Eigen::Vector3d &acc,
                               const Eigen::Quaterniond &rot,
                               wall_time ts)
{
    impl->omega = omega;
    impl->acc = acc;
    impl->rot = rot;

    measurementUpdated(ts);
}

ImuSensor::~ImuSensor()
{

}
