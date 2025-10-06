#include "impl/gripper.hxx"
#include <iostream>

using namespace XBot;

Gripper::Gripper(std::string name):
    Sensor(name)
{
    impl = std::make_unique<Impl>();   
}

void Gripper::Impl::process_callbacks(bool motion_ended, wall_time t)
{
    current_time = t;
    bool clear_cb = false;

    for(auto& [to, cb] : callbacks)
    {
        if(motion_ended || current_time >= to)
        {
            cb(!motion_ended);  // true -> timeout occurred
            clear_cb = true;
        }
    }

    if(clear_cb)
    {
        callbacks.clear();
    }
}

double Gripper::getClosure() const
{
    return impl->closure;
}

double Gripper::getEffort() const
{
    return impl->effort;
}

void Gripper::setClosureReference(double q, ClosureResultFn callback, double timeout)
{
    impl->closure_ref = q;
    impl->effort_ref.reset();

    auto to_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout)
    );

    if(callback)
    {
        impl->callbacks.emplace_back(
            impl->current_time + to_ns,
            callback
        );
    }
}

void Gripper::setEffortReference(double q, ClosureResultFn callback, double timeout)
{
    impl->effort_ref = q;
    impl->closure_ref.reset();
    
    auto to_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout)
    );

    if(callback)
    {
        impl->callbacks.emplace_back(
            impl->current_time + to_ns,
            callback
        );
    }
}

int Gripper::getNumPendingCallbacks() const
{
    return impl->callbacks.size();
}

std::optional<double> Gripper::getClosureReference() const
{
    return impl->closure_ref;
}

std::optional<double> Gripper::getEffortReference() const
{
    return impl->effort_ref;
}

void Gripper::setMeasurement(double closure, double effort, bool motion_ended, wall_time ts)
{
    impl->closure = closure;
    impl->effort = effort;

    impl->process_callbacks(motion_ended, ts);

    measurementUpdated(ts);
}

Gripper::~Gripper()
{
}
