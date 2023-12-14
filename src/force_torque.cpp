#include "impl/force_torque.hxx"

using namespace XBot;

ForceTorqueSensor::Impl::Impl()
{
    wrench.setZero();
}

ForceTorqueSensor::ForceTorqueSensor(string_const_ref name):
    Sensor(name)
{
    impl = std::make_unique<Impl>();
}

Eigen::Vector6d ForceTorqueSensor::getWrench() const
{
    return impl->wrench;
}

Eigen::Vector3d ForceTorqueSensor::getForce() const
{
    return impl->wrench.head<3>();
}

Eigen::Vector3d ForceTorqueSensor::getTorque() const
{
    return impl->wrench.tail<3>();
}

void ForceTorqueSensor::getWrench(Eigen::Vector6d &wrench) const
{
    wrench = impl->wrench;
}

void ForceTorqueSensor::getForce(Eigen::Vector3d &force) const
{
    force = impl->wrench.head<3>();
}

void ForceTorqueSensor::getTorque(Eigen::Vector3d &torque) const
{
    torque = impl->wrench.tail<3>();
}

void ForceTorqueSensor::setMeasurement(const Eigen::Vector6d &wrench, wall_time ts)
{
    impl->wrench = wrench;

    measurementUpdated(ts);
}

ForceTorqueSensor::~ForceTorqueSensor()
{

}
