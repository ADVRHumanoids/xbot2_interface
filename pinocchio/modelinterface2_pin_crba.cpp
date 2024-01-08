#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/crba.hpp>

using namespace XBot;

MatConstRef ModelInterface2Pin::computeInertiaMatrix() const
{
    if(!(_cached_computation & Crba))
    {
        pinocchio::crba(_mdl, _data, getJointPosition());

        _data.M.triangularView<Eigen::StrictlyLower>() =
            _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        _cached_computation |= Crba;
    }

    return _data.M;
}

