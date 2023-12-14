#ifndef FORCE_TORQUE_HXX
#define FORCE_TORQUE_HXX

#include <xbot2_interface/force_torque.h>

namespace XBot {

class ForceTorqueSensor::Impl
{

public:

    Impl();

    Eigen::Vector6d wrench;

private:

};

}


#endif // FORCE_TORQUE_HXX
