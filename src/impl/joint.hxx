#ifndef JOINT_HXX
#define JOINT_HXX

#include <xbot2_interface/types.h>

#include "../joint.h"
#include "state.hxx"



namespace XBot {

class Joint::Impl
{

public:

    XBOT_DECLARE_SMART_PTR(Joint)

    Impl(detail::StateView sv, detail::CommandView cv);

private:

    int _type;

    detail::StateView _state;

    detail::CommandView _cmd;


};

}

#endif // JOINT_HXX
