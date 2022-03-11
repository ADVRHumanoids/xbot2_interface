#include "robotinterface2_ros.h"
#include <xbot2_interface/common/plugin.h>

using namespace XBot;

RobotInterface2Ros::RobotInterface2Ros(std::unique_ptr<XBotInterface2> model):
    RobotInterface2(std::move(model)),
    _nh("xbotcore"),
    _js_received(false)
{
    _nh.setCallbackQueue(&_cbq);

    _js_sub = _nh.subscribe("joint_states", 1,
                            &RobotInterface2Ros::on_js_recv, this,
                            ros::TransportHints().udp().tcpNoDelay());

    auto timeout = ros::Time::now() + ros::Duration(5.0);
    while(!_js_received && ros::Time::now() < timeout)
    {
        _cbq.callAvailable();
        ros::Duration(0.001).sleep();
    }

    if(!_js_received)
    {
        throw std::runtime_error("no joint message received from topic: " + _js_sub.getTopic());
    }



}

bool RobotInterface2Ros::sense()
{
    _js_received = false;
    _cbq.callAvailable();
    return _js_received;
}

bool RobotInterface2Ros::move()
{
    return true;
}

void RobotInterface2Ros::on_js_recv(xbot_msgs::JointStateConstPtr msg)
{
    _js_received = true;

    for(size_t i = 0; i < msg->name.size(); i++)
    {
        auto j = getJoint(msg->name[i]);

        if(!j || j->getNq() > 1)
        {
            // TBD continuous joints
            continue;
        }

        j->setJointPosition(msg->link_position[i]);
        j->setJointVelocity(msg->link_velocity[i]);
        j->setJointEffort(msg->effort[i]);
    }
}

RobotInterface2Ros::RosInit::RosInit()
{
    if(!ros::isInitialized())
    {
        int argc = 0;
        char ** argv = 0;
        ros::init(argc, argv,
                  "robotinterfaceros_node",
                  ros::init_options::AnonymousName|ros::init_options::NoSigintHandler);
    }
}

XBOT2_REGISTER_ROBOT_PLUGIN(RobotInterface2Ros, ros);
