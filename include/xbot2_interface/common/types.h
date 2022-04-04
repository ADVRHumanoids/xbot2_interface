#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <memory>

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

typedef Eigen::Ref<Eigen::VectorXd> VecRef;
typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;

typedef Eigen::Ref<Eigen::CtrlModeVector> CtrlModeVectorRef;
typedef Eigen::Ref<const Eigen::CtrlModeVector> CtrlModeVectorConstRef;

typedef Eigen::Ref<Eigen::MatrixXd> MatRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

typedef std::string& string_ref;
typedef const std::string& string_const_ref;

Eigen::Scalard from_value(double value);

namespace ControlMode {
    enum Type
    {
        None = 0,
        Position = 1,
        Velocity = 2,
        Effort = 4,
        Stiffness = 8,
        Damping = 16
    };
}

struct JointInfo
{
    int id;
    int iq;
    int iv;
    int nq;
    int nv;

    JointInfo();
};

}



#endif // TYPES_H
