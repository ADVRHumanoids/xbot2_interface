#ifndef TYPES_H
#define TYPES_H

#include <memory>
#include <chrono>
#include <bitset>

#include <Eigen/Dense>

#include "visibility.h"

#define XBOT_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr;

namespace Eigen {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 1, 1> Scalard;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> CtrlModeVector;

}

namespace XBot {

inline namespace v2 {

typedef Eigen::Ref<Eigen::VectorXd> VecRef;
typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;

typedef Eigen::Ref<Eigen::CtrlModeVector> CtrlModeVectorRef;
typedef Eigen::Ref<const Eigen::CtrlModeVector> CtrlModeVectorConstRef;

typedef Eigen::Ref<Eigen::MatrixXd> MatRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

typedef std::string& string_ref;
typedef const std::string& string_const_ref;

typedef std::chrono::system_clock::time_point wall_time;

typedef std::unordered_map<std::string, double> JointNameMap;

XBOT2IFC_API Eigen::Scalard from_value(double value);

class ControlMode {

public:

    enum Type
    {
        NONE = 0,
        POSITION = 1,
        VELOCITY = 2,
        EFFORT = 4,
        STIFFNESS = 8,
        DAMPING = 16,
        ALL = 31
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

struct XBOT2IFC_API NotImplemented : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

struct XBOT2IFC_API JointInfo
{
    int id;
    int iq;
    int iv;
    int nq;
    int nv;

    JointInfo();
};

}

}



#endif // TYPES_H
