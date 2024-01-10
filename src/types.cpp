#include <xbot2_interface/common/types.h>

using namespace XBot;

Eigen::Scalard XBot::from_value(double value)
{
    return Eigen::Scalard{value};
}

std::pair<int, int> JointInfo::iqv() const
{
    return {iq, iv};
}

std::pair<int, int> JointInfo::nqv() const
{
    return {nq, nv};
}

std::pair<int, int> JointInfo::inq() const
{
    return {iq, nq};
}

std::pair<int, int> JointInfo::inv() const
{
    return {iv, nv};
}

JointInfo::JointInfo()
    : id(-1)
    , iq(-1)
    , iv(-1)
    , nq(-1)
    , nv(-1)
    , passive(false)

{

}

ControlMode::ControlMode(Type type):
    _type(type)
{

}

ControlMode::ControlMode(std::string name, Type type):
    _name(name), _type(type)
{

}

ControlMode::operator Type() const
{
    return _type;
}

ControlMode::Type ControlMode::type() const
{
    return _type;
}

ControlMode ControlMode::Position()
{
    return ControlMode("Position", POSITION);
}

ControlMode ControlMode::Velocity()
{
    return ControlMode("Velocity", POSITION);
}

ControlMode ControlMode::Effort()
{
    return ControlMode("Effort", EFFORT);
}

ControlMode ControlMode::Stiffness()
{
    return ControlMode("Stiffness", STIFFNESS);
}

ControlMode ControlMode::Damping()
{
    return ControlMode("Damping", DAMPING);
}

ControlMode ControlMode::Idle()
{
    return None();
}

ControlMode ControlMode::None()
{
    return ControlMode("None", NONE);
}

ControlMode ControlMode::FromBitset(Bitset bitset)
{
    return ControlMode(Type(bitset.to_ulong()));
}

ControlMode::Bitset ControlMode::AsBitset(const ControlMode &ctrl_mode)
{
    return Bitset(ctrl_mode._type);
}

ControlMode ControlMode::Impedance()
{
    return ControlMode("Impedance", Type(STIFFNESS|DAMPING));
}

ControlMode ControlMode::PosImpedance()
{
    return ControlMode("Position+Impedance", Type(POSITION|STIFFNESS|DAMPING));
}

ControlMode ControlMode::operator+(ControlMode other) const
{
    ControlMode ret;
    ret._type = static_cast<Type>(_type | other._type);
    ret._name = _name + "+" + other._name;
    return ret;
}

bool ControlMode::operator==(ControlMode ctrl_mode) const
{
    return _type == ctrl_mode._type;
}

bool ControlMode::isPositionEnabled() const
{
    return _type & POSITION;
}

bool ControlMode::isVelocityEnabled() const
{
    return _type & VELOCITY;
}

bool ControlMode::isEffortEnabled() const
{
    return _type & EFFORT;
}

bool ControlMode::isStiffnessEnabled() const
{
    return _type & STIFFNESS;
}

bool ControlMode::isDampingEnabled() const
{
    return _type & DAMPING;
}

const std::string &ControlMode::getName() const
{
    return _name;
}
