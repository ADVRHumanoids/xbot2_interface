#include "robotinterface2_ros.h"
#include "config_from_param.hpp"
#include <xbot2_interface/common/plugin.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot;

RobotInterface2Ros::RobotInterface2Ros(std::unique_ptr<ModelInterface> model):
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

    ROS_INFO("started listening to joint states..");
    int niter = 0;
    while(!_js_received && niter++ < 100)
    {
        _cbq.callAvailable();
        ros::Duration(0.01).sleep();
    }

    if(!_js_received)
    {
        throw std::runtime_error("no joint message received from topic: " + _js_sub.getTopic());
    }

    ROS_INFO("got joint states!");

    // imu
    auto imu_map = getImuNonConst();

    for(auto [name, imu] : imu_map)
    {
        auto cb = [imu](const sensor_msgs::ImuConstPtr& msg)
        {
            wall_time ts = wall_time() +
                           std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nsec);

            imu->setMeasurement(
                Eigen::Vector3d::Map(&msg->angular_velocity.x),
                Eigen::Vector3d::Map(&msg->linear_acceleration.x),
                Eigen::Quaterniond(&msg->orientation.x),
                ts);
        };

        auto sub = _nh.subscribe<sensor_msgs::Imu>("imu/" + name,
                                                   1,
                                                   cb,
                                                   nullptr,
                                                   ros::TransportHints().tcpNoDelay(true));

        _subs.push_back(sub);
    }

    // ft
    auto ft_map = getForceTorqueNonConst();

    for(auto [name, ft] : ft_map)
    {
        auto cb = [ft](const geometry_msgs::WrenchStampedConstPtr& msg)
        {
            wall_time ts = wall_time() +
                           std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nsec);

            ft->setMeasurement(
                Eigen::Vector6d::Map(&msg->wrench.force.x),
                ts);
        };

        auto sub = _nh.subscribe<geometry_msgs::WrenchStamped>(
            "ft/" + name,
            1,
            cb,
            nullptr,
            ros::TransportHints().tcpNoDelay(true));

        _subs.push_back(sub);
    }
}

bool RobotInterface2Ros::sense_impl()
{
    _js_received = false;
    _cbq.callAvailable();
    return _js_received;
}

bool RobotInterface2Ros::move_impl()
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
        if(ctrl == ControlMode::NONE)
        {
            continue;
        }

        // fill msg
        cmd.name.push_back(j->getName());
        cmd.ctrl_mode.push_back(ctrl);
        cmd.position.push_back(j->getPositionReferenceMinimal().value());
        cmd.velocity.push_back(j->getVelocityReference().value());
        cmd.effort.push_back(j->getEffortReference().value());
        cmd.stiffness.push_back(j->getStiffnessDesired().value());
        cmd.damping.push_back(j->getDampingDesired().value());

        pub_cmd = true;

    }

    if(pub_cmd)
    {
        _cmd_pub.publish(cmd);
    }

    // handle base
    auto jfb = getUniversalJoint(0);
    if(jfb->getType() == urdf::Joint::FLOATING &&
            (jfb->getValidCommandMask()[0] & ControlMode::VELOCITY))
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
        j->setJointPositionMinimal(msg->link_position[i]);

        j->setPositionReferenceFeedbackMinimal(msg->position_reference[i]);

        j->setJointVelocity(msg->link_velocity[i]);

        j->setJointEffort(msg->effort[i]);

        j->setStiffnessFeedback(msg->stiffness[i]);

        j->setDampingFeedback(msg->damping[i]);


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
