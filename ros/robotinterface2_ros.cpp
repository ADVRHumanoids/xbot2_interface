#include "robotinterface2_ros.h"
#include <xbot2_interface/common/plugin.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot;

RobotInterface2Ros::RobotInterface2Ros(std::unique_ptr<XBotInterface> model):
    RobotInterface(std::move(model)),
    _nh("xbotcore"),
    _js_received(false)
{
    _nh.setCallbackQueue(&_cbq);

    _js_sub = _nh.subscribe("joint_states", 1,
                            &RobotInterface2Ros::on_js_recv, this,
                            ros::TransportHints().udp().tcpNoDelay());

    _cmd_pub = _nh.advertise<xbot_msgs::JointCommand>("command", 10);

    auto jfb = getJoint(0);
    if(jfb->getType() == urdf::Joint::FLOATING)
    {
        _base_cmd_pub = _nh.advertise<geometry_msgs::TwistStamped>(jfb->getChildLink() + "/cmd_vel", 1);
    }

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
    xbot_msgs::JointCommand cmd;
    const int nj = getJointNum();

    cmd.header.stamp = ros::Time::now();

    cmd.name.reserve(nj);
    cmd.position.reserve(nj);
    cmd.velocity.reserve(nj);
    cmd.effort.reserve(nj);
    cmd.stiffness.reserve(nj);
    cmd.damping.reserve(nj);
    cmd.ctrl_mode.reserve(nj);

    bool pub_cmd = false;

    for(int i = 0; i < getJointNum(); i++)
    {
        auto j = getUniversalJoint(i);

        // skip multi-dof
        if(j->getNv() > 1)
        {
            continue;
        }

        // get valid ctrl mask and clear it right after
        auto ctrl = ControlMode::Type(j->getValidCommandMask()[0]);

        j->clearCommandMask();

        // if no cmd was set, skip
        if(ctrl == ControlMode::None)
        {
            continue;
        }

        // fill msg
        cmd.name.push_back(j->getName());
        cmd.ctrl_mode.push_back(ctrl);
        cmd.position.push_back(j->getPositionReferenceMinimal().value());
        cmd.velocity.push_back(0.0);
        cmd.effort.push_back(0.0);
        cmd.stiffness.push_back(j->getStiffnessCmd().value());
        cmd.damping.push_back(0.0);

        pub_cmd = true;

    }

    if(pub_cmd)
    {
        _cmd_pub.publish(cmd);
    }

    // handle base
    auto jfb = getUniversalJoint(0);
    if(jfb->getType() == urdf::Joint::FLOATING &&
            (jfb->getValidCommandMask()[0] & ControlMode::Velocity))
    {
        geometry_msgs::TwistStamped basecmd;
        basecmd.header.stamp = cmd.header.stamp;

        Eigen::Affine3d T;
        Eigen::Vector6d v;
        jfb->forwardKinematics(jfb->getPositionReference(),
                               jfb->getVelocityReference(),
                               T, v);

        tf::twistEigenToMsg(v, basecmd.twist);

        _base_cmd_pub.publish(basecmd);
    }

    return true;
}

void RobotInterface2Ros::on_js_recv(xbot_msgs::JointStateConstPtr msg)
{
    _js_received = true;

    for(size_t i = 0; i < msg->name.size(); i++)
    {
        auto j = getUniversalJoint(msg->name[i]);

        if(!j)
        {
            continue;
        }

        // skip multi-dof
        if(j->getNv() > 1)
        {
            continue;
        }

        // nq could be =2 for SO(2), so we use "minimal" versions
        j->setJointPositionMinimal(from_value(msg->link_position[i]));
        j->setPositionReferenceFeedbackMinimal(from_value(msg->position_reference[i]));

        j->setJointVelocity(from_value(msg->link_velocity[i]));

        j->setJointEffort(from_value(msg->effort[i]));

        j->setStiffnessFeedback(from_value(msg->stiffness[i]));


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
