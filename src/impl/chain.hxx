#ifndef CHAIN_HXX
#define CHAIN_HXX

#include <Eigen/Dense>

#include <xbot2_interface/types.h>
#include "state.hxx"


namespace XBot {

class Chain
{

public:

    XBOT_DECLARE_SMART_PTR(Chain);

private:

    detail::StateView _state;

    detail::CommandView _cmd;


};

}

#endif // CHAIN_HXX
