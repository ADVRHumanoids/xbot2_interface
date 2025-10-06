#ifndef XBOT2IFC_GRIPPER_H
#define XBOT2IFC_GRIPPER_H

#include <xbot2_interface/sensor.h>
#include <optional>

namespace XBot {
inline namespace v2 {

class XBOT2IFC_API Gripper : public Sensor
{

public:

    typedef std::function<void(bool)> ClosureResultFn;

    XBOT_DECLARE_SMART_PTR(Gripper)

    Gripper(std::string name);

    double getClosure() const;

    double getEffort() const;

    void setClosureReference(double q, 
        ClosureResultFn callback = ClosureResultFn(), 
        double timeout = -1.0);

    void setEffortReference(double q, 
        ClosureResultFn callback = ClosureResultFn(), 
        double timeout = -1.0);

    int getNumPendingCallbacks() const;

    std::optional<double> getClosureReference() const;

    std::optional<double> getEffortReference() const;

    // used internally to update state
    void setMeasurement(double closure, double effort, bool motion_ended,
                        wall_time ts);

    virtual ~Gripper();


private:

    class Impl;
    std::unique_ptr<Impl> impl;

};

    
}  
}


#endif