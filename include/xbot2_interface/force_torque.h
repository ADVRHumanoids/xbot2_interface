#ifndef XBOT2IFC_FORCE_TORQUE_H
#define XBOT2IFC_FORCE_TORQUE_H

#include "sensor.h"

namespace XBot {

inline namespace v2 {

class XBOT2IFC_API ForceTorqueSensor : public Sensor {

public:

    XBOT_DECLARE_SMART_PTR(ForceTorqueSensor)

    ForceTorqueSensor(string_const_ref name);

    Eigen::Vector6d getWrench() const;

    Eigen::Vector3d getForce() const;

    Eigen::Vector3d getTorque() const;

    void getWrench(Eigen::Vector6d& wrench) const;

    void getForce(Eigen::Vector3d& force) const;

    void getTorque(Eigen::Vector3d& torque) const;

    void setMeasurement(const Eigen::Vector6d& wrench,
                        wall_time ts);

    ~ForceTorqueSensor();

protected:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

}

#endif // FORCE_TORQUE_H
