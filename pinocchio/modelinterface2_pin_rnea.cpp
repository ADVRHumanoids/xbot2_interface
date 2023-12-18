#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/rnea.hpp>

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

VecConstRef ModelInterface2Pin::computeGravityCompensation() const
{
    if(!(_cached_computation & Gcomp))
    {

        _tmp.rnea = pinocchio::computeGeneralizedGravity(_mdl, _data,
                                                         getJointPosition());


        _cached_computation |= Gcomp;

    }

    return _tmp.rnea;
}
