#include <xbot2_interface/common/types.h>

using namespace XBot;

Eigen::Scalard XBot::from_value(double value)
{
    return Eigen::Scalard{value};
}

JointInfo::JointInfo():
    id(-1),
    iq(-1),
    iv(-1),
    nq(-1),
    nv(-1)

{
}
