#include "robotinterface2_ros.h"
#include "config_from_param.hpp"
#include <xbot2_interface/common/plugin.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>

using namespace XBot;

RobotInterface2Ros::RobotInterface2Ros(std::unique_ptr<ModelInterface> model):
    RobotInterface(std::move(model)),
    _js_received(false)
{
    RosInit();
    auto time_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    _node = rclcpp::Node::make_shared("robot_ifc_" + std::to_string(time_ns));

    _exe = rclcpp::executors::SingleThreadedExecutor::make_unique();
    _exe->add_node(_node);

    /* Connect to joint states */
    auto js_sub = _node->create_subscription<JointState>("xbotcore/joint_states",
                                                         rclcpp::SensorDataQoS().keep_last(1),
                                                         [this](JointState::ConstSharedPtr msg) {
                                                             on_js_recv(msg);
                                                         });

    _subs.push_back(js_sub);

    /* Connect to command topic */
    _cmd_pub = _node->create_publisher<JointCommand>("xbotcore/command",
                                                     rclcpp::SensorDataQoS().keep_last(1));

    auto jfb = getJoint(0);
    if(jfb->getType() == urdf::Joint::FLOATING)
    {
        _base_cmd_pub = _node->create_publisher<Twist>(jfb->getChildLink() + "/cmd_vel", 1);
    }

    RCLCPP_INFO(_node->get_logger(), "started listening to joint states..");
    int niter = 0;
    while(!_js_received && niter++ < 100)
    {
        _exe->spin_some();
        _node->get_clock()->sleep_for(100ms);
    }

    if(!_js_received)
    {
        throw std::runtime_error("no joint message received from topic: " + std::string(js_sub->get_topic_name()));
    }

    RCLCPP_INFO(_node->get_logger(), "got joint states!");

    // imu
    auto imu_map = getImuNonConst();

    for(auto [name, imu] : imu_map)
    {
	    auto imu0 = imu;  // some compilers cannot capture structured bindings...

        auto cb = [imu0](Imu::ConstSharedPtr msg)
        {
            wall_time ts = wall_time() +
                           std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nanosec);

            imu0->setMeasurement(
                Eigen::Vector3d::Map(&msg->angular_velocity.x),
                Eigen::Vector3d::Map(&msg->linear_acceleration.x),
                Eigen::Quaterniond(&msg->orientation.x),
                ts);
        };

        auto sub = _node->create_subscription<Imu>("xbotcore/imu/" + name,
                                                   rclcpp::SensorDataQoS().keep_last(1),
                                                   cb);

        _subs.push_back(sub);
    }

    // ft
    auto ft_map = getForceTorqueNonConst();

    for(auto [name, ft] : ft_map)
    {
	    auto ft0 = ft;  // some compilers cannot capture structured bindings...
	    
        auto cb = [ft0](WrenchStamped::ConstSharedPtr msg)
        {
            wall_time ts = wall_time() +
                           std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nanosec);

            ft0->setMeasurement(
                Eigen::Vector6d::Map(&msg->wrench.force.x),
                ts);
        };

        auto sub = _node->create_subscription<WrenchStamped>("xbotcore/ft/" + name,
                                             rclcpp::SensorDataQoS().keep_last(1),
                                             cb);

        _subs.push_back(sub);
    }

    // grippers
    auto gripper_map = getGripper();

    for(auto [name, gr] : gripper_map)
    {
	    auto gr0 = gr;  // some compilers cannot capture structured bindings...

        struct _GrHistory {
            wall_time next_meas_time;
            double last_closure = -666;
        };

        auto _gr_history = std::make_shared<_GrHistory>();
	    
        auto cb = [gr0, _gr_history](JointStateSM::ConstSharedPtr msg)
        {
            wall_time ts = wall_time() +
                           std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nanosec);

            double closure = msg->position[0];

            bool motion_ended = false;

            if(ts > _gr_history->next_meas_time)
            {
                // force two consecutive measures at least 666ms apart
                // and while there are pending callbacks
                if(gr0->getNumPendingCallbacks() == 0)
                {
                    _gr_history->last_closure = -666;
                }

                _gr_history->next_meas_time = ts + 666ms;
                
                if(std::fabs(_gr_history->last_closure - closure) < 0.01)
                {
                    motion_ended = true;
                }

                _gr_history->last_closure = closure;
            }

            gr0->setMeasurement(
                msg->position[0], msg->effort[0], motion_ended,
                ts);
        };

        auto sub = _node->create_subscription<JointStateSM>("xbotcore/gripper/" + name + "/state",
                                             rclcpp::SensorDataQoS().keep_last(1),
                                             cb);

        _subs.push_back(sub);

        auto pub = _node->create_publisher<JointStateSM>("xbotcore/gripper/" + name + "/command",
                                            1);

        _gripper_cmd[gr] = pub;
    }

    
}

bool RobotInterface2Ros::sense_impl()
{
    _js_received = false;
    _exe->spin_all(0s);
    return _js_received;
}

bool RobotInterface2Ros::move_impl()
{
    JointCommand cmd;
    const int nj = getJointNum();

    cmd.header.stamp = _node->get_clock()->now();

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
        _cmd_pub->publish(cmd);
    }

    // handle base
    auto jfb = getUniversalJoint(0);
    if(jfb->getType() == urdf::Joint::FLOATING &&
            (jfb->getValidCommandMask()[0] & ControlMode::VELOCITY))
    {
        Eigen::Affine3d T;
        Eigen::Vector6d v;
        jfb->forwardKinematics(jfb->getPositionReference(),
                               jfb->getVelocityReference(),
                               T, v);

        _base_cmd_pub->publish(tf2::toMsg(v));
    }

    // handle gripprs
    for(auto [gr, pub] : _gripper_cmd)
    {
        JointStateSM msg;
        msg.header.stamp = cmd.header.stamp;
        msg.name = {gr->getName()};
        if(auto cl = gr->getClosureReference())
        {
            msg.position = {*cl};
        }
        if(auto ef = gr->getEffortReference())
        {
            msg.effort = {*ef};
        }
        pub->publish(msg);
    }

    return true;
}

void RobotInterface2Ros::on_js_recv(JointState::ConstSharedPtr msg)
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

    wall_time stamp = wall_time(msg->header.stamp.sec * 1s + msg->header.stamp.nanosec * 1ns);
    markAsUpdated(stamp);
}

RobotInterface2Ros::RosInit::RosInit()
{
    // Ros init
    if(!rclcpp::ok())
    {
        std::cout << "calling rclcpp::init \n";
        int argc = 0;
        rclcpp::init(argc, nullptr,
                     rclcpp::InitOptions(),
                     rclcpp::SignalHandlerOptions::None);
    }
}

XBOT2_REGISTER_ROBOT_PLUGIN(RobotInterface2Ros, ros2);

