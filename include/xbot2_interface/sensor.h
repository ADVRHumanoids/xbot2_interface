#ifndef SENSOR_H
#define SENSOR_H

#include "common/types.h"

namespace XBot {

inline namespace v2 {

class XBOT2IFC_API Sensor {

public:

    XBOT_DECLARE_SMART_PTR(Sensor)

    Sensor(string_const_ref name);

    string_const_ref getName() const;

    bool isUpdated() const;

    wall_time getTimestamp() const;

    void clear();

    ~Sensor();

protected:

    void measurementUpdated(wall_time ts);

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

}

#endif // SENSOR_H
