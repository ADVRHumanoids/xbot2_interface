#ifndef IMU_HXX
#define IMU_HXX

#include <xbot2_interface/imu.h>

namespace XBot {

class ImuSensor::Impl
{

public:

    Impl();

    Eigen::Vector3d omega, acc;
    Eigen::Quaterniond rot;

private:

};

}


#endif // IMU_HXX
