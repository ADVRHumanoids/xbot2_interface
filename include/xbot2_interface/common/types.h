#ifndef XBOT2IFC_TYPES_H
#define XBOT2IFC_TYPES_H

#include <memory>
#include <chrono>
#include <unordered_map>

#include <Eigen/Dense>

#include "visibility.h"
#include "control_mode.h"

#define XBOT_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr;

namespace Eigen {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
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
typedef std::unordered_map<std::string, ControlMode> CtrlModeMap;
typedef std::unordered_map<std::string, ControlMode::Type> CtrlModeTypeMap;

XBOT2IFC_API Eigen::Scalard from_value(double value);

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
    bool passive;

    std::pair<int, int> iqv() const;
    std::pair<int, int> nqv() const;
    std::pair<int, int> inq() const;
    std::pair<int, int> inv() const;

    JointInfo();
};

enum class Sync {
    MotorSide,
    LinkSide
};

}

}



#endif // TYPES_H
