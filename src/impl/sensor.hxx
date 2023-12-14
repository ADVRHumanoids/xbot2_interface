#ifndef SENSOR_HXX
#define SENSOR_HXX

#include <xbot2_interface/sensor.h>

namespace XBot {

class Sensor::Impl
{

public:

    Impl(string_const_ref name);

    std::string name;
    bool is_updated;
    wall_time ts;

private:

};

}


#endif // SENSOR_HXX
