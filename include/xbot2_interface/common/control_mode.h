#ifndef XBOT2IFC_CONTROL_MODE_H
#define XBOT2IFC_CONTROL_MODE_H

#include "visibility.h"
#include <string>
#include <bitset>

namespace XBot {
inline namespace v2 {

class XBOT2IFC_API ControlMode {

public:

    enum Type
    {
        NONE = 0,
        POSITION = 1,
        VELOCITY = 2,
        EFFORT = 4,
        STIFFNESS = 8,
        DAMPING = 16,
        ACCELERATION = 32,
        ALL = 63
    };

    /**
     * @brief Representation of a control mode as a bitset
     *
     * Bit 0 -> position
     * Bit 1 -> velocity
     * Bit 2 -> effort
     * Bit 3 -> stiffness
     * Bit 4 -> damping
     */
    typedef std::bitset<5> Bitset;

    ControlMode(Type type = NONE);

    ControlMode(std::string name, Type type = NONE);

    operator Type() const;

    Type type() const;

    ControlMode operator+(ControlMode other) const;

    bool operator==(ControlMode ctrl_mode) const;

    bool isPositionEnabled() const;

    bool isVelocityEnabled() const;

    bool isEffortEnabled() const;

    bool isStiffnessEnabled() const;

    bool isDampingEnabled() const;

    const std::string& getName() const;

    static ControlMode Position();

    static ControlMode Velocity();

    static ControlMode Effort();

    static ControlMode PosImpedance();

    static ControlMode Impedance();

    static ControlMode Stiffness();

    static ControlMode Damping();

    [[deprecated("Use None() instead")]]
    static ControlMode Idle();

    static ControlMode None();

    static ControlMode FromBitset(Bitset bitset);

    static Bitset AsBitset(const ControlMode& ctrl_mode);

private:

    std::string _name;
    Type _type;
};

XBOT2IFC_API ControlMode::Type operator|(ControlMode::Type a, ControlMode::Type b);
XBOT2IFC_API ControlMode::Type operator&(ControlMode::Type a, ControlMode::Type b);

}
}

#endif // CONTROL_MODE_H
