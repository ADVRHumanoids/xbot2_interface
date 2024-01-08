#ifndef ROBOTINTERFACE2_HXX
#define ROBOTINTERFACE2_HXX

#include <map>
#include <string>
#include <string_view>

#include <xbot2_interface/robotinterface2.h>

#include "state.hxx"

namespace XBot {

class RobotInterface::Impl
{

public:

    Impl(RobotInterface& api,
         std::unique_ptr<ModelInterface> model);

    friend RobotInterface;

private:

    // api
    RobotInterface& _api;

    // model
    std::shared_ptr<ModelInterface> _model;

};

}

#endif // ROBOTINTERFACE2_HXX
