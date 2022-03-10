#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Dense>
#include <memory>

#define XBOT_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr;

namespace XBot {

typedef Eigen::Ref<Eigen::VectorXd> VecRef;
typedef Eigen::Ref<const Eigen::VectorXd> VecConstRef;

typedef Eigen::Ref<Eigen::MatrixXd> MatRef;
typedef Eigen::Ref<const Eigen::MatrixXd> MatConstRef;

}

#endif // TYPES_H
