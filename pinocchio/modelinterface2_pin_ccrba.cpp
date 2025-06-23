#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/centroidal.hpp>

using namespace XBot;

MatConstRef ModelInterface2Pin::computeCentroidalMomentumMatrix() const
{
    if(!(_cached_computation & CCrba))
    {
        pinocchio::ccrba(_mdl, _data, getJointPosition(), getJointVelocity());

        _cached_computation |= CCrba;
    }

    return _data.Ag;
}

Eigen::Vector6d ModelInterface2Pin::computeCentroidalMomentum() const
{
    if(!(_cached_computation & CCrba))
    {
        return pinocchio::computeCentroidalMomentum(_mdl, _data);
    }

    return _data.hg;
}

Eigen::Vector6d ModelInterface2Pin::computeCentroidalMomentumdotTimesV() const
{
    if(!(_cached_computation & CCrba))
    {
        return pinocchio::computeCentroidalMomentumTimeVariation(_mdl, _data_no_acc);
    }

    return _data.dhg;
}
