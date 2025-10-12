#ifndef XBOT2IFC_IMU_H
#define XBOT2IFC_IMU_H

#include "sensor.h"

namespace XBot {

inline namespace v2 {

class XBOT2IFC_API ImuSensor : public Sensor {

public:

    XBOT_DECLARE_SMART_PTR(ImuSensor)

    ImuSensor(string_const_ref name);

    Eigen::Vector3d getAngularVelocity() const;

    Eigen::Vector3d getLinearAcceleration() const;

    Eigen::Quaterniond getOrientation() const;

    void getAngularVelocity(Eigen::Vector3d& omega) const;

    void getLinearAcceleration(Eigen::Vector3d& acc) const;

    void getOrientation(Eigen::Quaterniond& rot) const;

    void setMeasurement(const Eigen::Vector3d& omega,
                        const Eigen::Vector3d& acc,
                        const Eigen::Quaterniond& rot,
                        wall_time ts);

    ~ImuSensor();

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

}

#endif // IMU_H
