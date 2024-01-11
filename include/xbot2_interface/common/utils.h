#ifndef XBOT2IFC_UTILS_H
#define XBOT2IFC_UTILS_H

#include <Eigen/Dense>
#include "types.h"

namespace urdf {
class ModelInterface;
}

namespace srdf {
class Model;
}

namespace XBot::Utils {

inline namespace v2 {

XBOT2IFC_API Eigen::Matrix3d rpyToRotationMatrix(Eigen::Ref<const Eigen::Vector3d> rpy);

XBOT2IFC_API Eigen::Matrix3d rpyJacobian(Eigen::Ref<const Eigen::Vector3d> rpy);

XBOT2IFC_API Eigen::Vector3d rotationMatrixToRpy(const Eigen::Matrix3d& R);

XBOT2IFC_API Eigen::Matrix3d skew(const Eigen::Vector3d& r);

XBOT2IFC_API Eigen::Matrix6d adjointFromRotation(const Eigen::Matrix3d &R);

template <typename Mat>
void changeRefPoint(Eigen::MatrixBase<Mat>& J_or_vel,
                    const Eigen::Vector3d& p)
{
    // v(P) = v(0) + om x (P - O)
    Eigen::Matrix3d S = skew(p);
    J_or_vel.template topRows<3>().noalias() -= S * J_or_vel.template bottomRows<3>();
}

XBOT2IFC_API const Eigen::Vector6d& rotate(Eigen::Vector6d& vel,
                              const Eigen::Matrix3d& R);

template <typename Mat, typename MatOutput>
const Eigen::MatrixBase<MatOutput>& rotate(const Eigen::MatrixBase<Mat>& J,
                                     const Eigen::Matrix3d& R,
                                     Eigen::MatrixBase<MatOutput>& output)
{
    output.resize(J.rows(), J.cols());
    output.template topRows<3>().noalias() = R * J.template topRows<3>();
    output.template bottomRows<3>().noalias() = R * J.template bottomRows<3>();
    return output;
}

/**
 * @brief Computes an orientation error between two frames such that
 * an angular velocity K*e (K > 0) brings "actual" towards "ref"
 *
 * @param ref Reference orientation
 * @param actual Current orientation
 * @param error The orientation error between ref and actual
 * @return void
 */
XBOT2IFC_API void computeOrientationError(const Eigen::Matrix3d &ref,
                                          const Eigen::Matrix3d &actual,
                                          Eigen::Vector3d &error);

XBOT2IFC_API Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d &ref,
                                                     const Eigen::Matrix3d &actual);

XBOT2IFC_API Eigen::Vector6d computePoseError(const Eigen::Affine3d &ref,
                                              const Eigen::Affine3d &actual);

XBOT2IFC_API std::string urdfToString(const urdf::ModelInterface& urdf);

XBOT2IFC_API std::string srdfToString(const urdf::ModelInterface &urdf, const srdf::Model &srdf);

XBOT2IFC_API double quinticSpline(double tau);

XBOT2IFC_API double quinticSpline(double t0, double tf, double time);

XBOT2IFC_API std::tuple<double, double, double> quinticSplineDerivatives(double tau);

XBOT2IFC_API std::tuple<double, double, double> quinticSplineDerivatives(double t0, double tf, double time);

template <typename SignalType>
/**
 * @brief SecondOrderFilter implements a canonical continuous-time
 * second order filter with transfer function
 *                   1
 * P(s) =  -----------------------,  w = natural frequency, eps = damping ratio
 *         (s/w)^2 + 2*eps/w*s + 1
 *
 * and discretized according to a trapezoidal (aka Tustin) scheme. This yields
 * a difference equation of the following form:
 *
 *      a0*y + a1*yd + a2*ydd = u + b1*ud + b2*udd
 *
 * where yd = y(k-1), ydd = y(k-2) and so on (d = delayed).
 */
class SecondOrderFilter {

public:

    XBOT_DECLARE_SMART_PTR(SecondOrderFilter)

    SecondOrderFilter():
        _omega(1.0),
        _eps(0.8),
        _ts(0.01),
        _reset_has_been_called(false)
    {
        computeCoeff();
    }

    SecondOrderFilter(double omega, double eps, double ts, const SignalType& initial_state):
        _omega(omega),
        _eps(eps),
        _ts(ts),
        _reset_has_been_called(false)
    {
        computeCoeff();
        reset(initial_state);
    }

    void reset(const SignalType& initial_state)
    {
        _reset_has_been_called = true;
        _u = initial_state;
        _y = initial_state;
        _yd = initial_state;
        _ydd = initial_state;
        _udd = initial_state;
        _ud = initial_state;
    }

    const SignalType& process(const SignalType& input)
    {
        if(!_reset_has_been_called)
        {
            reset(input*0);
        }

        _ydd = _yd;
        _yd = _y;
        _udd = _ud;
        _ud = _u;

        _u = input;
        _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

        return _y;
    }

    const SignalType& getOutput() const
    {
        return _y;
    }

    void setOmega(double omega)
    {
        _omega = omega;
        computeCoeff();
    }

    double getOmega()
    {
        return _omega;
    }

    void setDamping(double eps)
    {
        _eps = eps;
        computeCoeff();
    }

    double getDamping()
    {
        return _eps;
    }

    void setTimeStep(double ts)
    {
        _ts = ts;
        computeCoeff();
    }

    double getTimeStep()
    {
        return _ts;
    }

private:

    void computeCoeff()
    {
        _b1 = 2.0;
        _b2 = 1.0;

        _a0 = 1.0 + 4.0*_eps/(_omega*_ts) + 4.0/std::pow(_omega*_ts, 2.0);
        _a1 = 2 - 8.0/std::pow(_omega*_ts, 2.0);
        _a2 = 1.0 + 4.0/std::pow(_omega*_ts, 2.0) - 4.0*_eps/(_omega*_ts);
    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;

    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};

}

}

#endif // UTILS_H
