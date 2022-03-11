#ifndef ROBOTINTERFACE2_HXX
#define ROBOTINTERFACE2_HXX

#include <map>
#include <string>
#include <string_view>

#include <xbot2_interface/robotinterface2.h>


namespace XBot {

class RobotInterface2::Impl
{

public:

    Impl(RobotInterface2& api,
         std::unique_ptr<XBotInterface2> model);

    friend RobotInterface2;

private:

    // api
    RobotInterface2& _api;

    // model
    std::unique_ptr<XBotInterface2> _model;


};

}

#endif // ROBOTINTERFACE2_HXX
