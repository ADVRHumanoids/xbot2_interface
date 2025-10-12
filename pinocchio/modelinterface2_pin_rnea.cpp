#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

using namespace XBot;

VecConstRef ModelInterface2Pin::computeInverseDynamics() const
{
    if(!(_cached_computation & Rnea))
    {

        _tmp.rnea = pinocchio::rnea(_mdl, _data,
                                    getJointPosition(),
                                    getJointVelocity(),
                                    getJointAcceleration());


        _cached_computation |= Rnea;

    }

    return _tmp.rnea;
}

void ModelInterface2Pin::computeInverseDynamicsDerivative(Eigen::MatrixXd& dtau_dq, Eigen::MatrixXd& dtau_dv, Eigen::MatrixXd& dtau_da)
{
    //Maybe here it is possible to cache also ccrba!
    if(!(_cached_computation & dRnea))
    {
        pinocchio::computeRNEADerivatives(_mdl, _data,
                                          getJointPosition(),
                                          getJointVelocity(),
                                          getJointAcceleration());

        _data.M.triangularView<Eigen::StrictlyLower>() = _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        _cached_computation |= dRnea;
    }

    dtau_dq = _data.dtau_dq;
    dtau_dv = _data.dtau_dv;

    dtau_da = _data.M;
}

VecConstRef ModelInterface2Pin::computeGravityCompensation() const
{
    if(!(_cached_computation & Gcomp))
    {

        _tmp.gcomp = pinocchio::computeGeneralizedGravity(_mdl, _data,
                                                         getJointPosition());


        _cached_computation |= Gcomp;

    }

    return _tmp.gcomp;
}

VecConstRef ModelInterface2Pin::computeNonlinearTerm() const
{
    if(!(_cached_computation & NonlinearEffects))
    {
        _tmp.h = pinocchio::nonLinearEffects(_mdl, _data,
                                             getJointPosition(), getJointVelocity());

        _cached_computation |= NonlinearEffects;

    }

    return _tmp.h;
}
