#ifndef XBOT2IFC_GRIPPER_HXX
#define XBOT2IFC_GRIPPER_HXX

#include <xbot2_interface/gripper.h>

namespace XBot {

class Gripper::Impl
{

public:

    Impl() = default;

    double closure = 0.0;
    double effort = 0.0;
    std::optional<double> closure_ref;
    std::optional<double> effort_ref;
    std::vector<std::pair<wall_time, ClosureResultFn>> callbacks;
    wall_time current_time;

    void process_callbacks(bool motion_ended, wall_time t);

private:

};

}

#endif