#include "impl/sensor.hxx"

using namespace  XBot;

Sensor::Sensor(string_const_ref name)
{
    impl = std::make_unique<Impl>(name);
}

string_const_ref Sensor::getName() const
{
    return impl->name;
}

bool Sensor::isUpdated() const
{
    return impl->is_updated;
}

wall_time Sensor::getTimestamp() const
{
    return impl->ts;
}

void Sensor::clear()
{
    impl->is_updated = false;
}

Sensor::~Sensor()
{

}

void Sensor::measurementUpdated(wall_time ts)
{
    impl->is_updated = true;
    impl->ts = ts;
}

Sensor::Impl::Impl(string_const_ref _name):
    name(_name),
    is_updated(false)
{

}
