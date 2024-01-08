#include "modelinterface2_pin.h"

#include <pinocchio/algorithm/aba.hpp>

using namespace XBot;

VecConstRef ModelInterface2Pin::computeForwardDynamics() const
{
    return pinocchio::aba(_mdl, _data,
                          getJointPosition(),
                          getJointVelocity(),
                          getJointEffort());
}

MatConstRef ModelInterface2Pin::computeInertiaInverse() const
{
    if(!(_cached_computation & Minv))
    {
        pinocchio::computeMinverse(_mdl, _data, getJointPosition());

        _data.Minv.triangularView<Eigen::StrictlyLower>() =
            _data.Minv.transpose().triangularView<Eigen::StrictlyLower>();


        _cached_computation |= Minv;
    }

    return Eigen::Map<Eigen::MatrixXd>(_data.Minv.data(), _mdl.nv, _mdl.nv);
}
