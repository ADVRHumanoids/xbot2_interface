#include <xbot2_interface/common/types.h>

using namespace XBot;

Eigen::Scalard XBot::from_value(double value)
{
    return Eigen::Scalard{value};
}
