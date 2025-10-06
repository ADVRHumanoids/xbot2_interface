#ifndef ROBOTINTERFACE2_ROS_H
#define ROBOTINTERFACE2_ROS_H

#include <xbot2_interface/robotinterface2.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <xbot_msgs/msg/joint_command.hpp>
#include <xbot_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace xbot_msgs::msg;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using Imu = sensor_msgs::msg::Imu;
using JointStateSM = sensor_msgs::msg::JointState;

namespace XBot {

class RobotInterface2Ros : public RobotInterface
{

public:

    RobotInterface2Ros(std::unique_ptr<ModelInterface> model);

    bool sense_impl() override;

    bool move_impl() override;

private:

    struct RosInit
    {
        RosInit();
    };

    void on_js_recv(JointState::ConstSharedPtr msg);

    rclcpp::Node::SharedPtr _node;

    rclcpp::executors::SingleThreadedExecutor::UniquePtr _exe;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> _subs;

    rclcpp::Publisher<JointCommand>::SharedPtr _cmd_pub;

    rclcpp::Publisher<Twist>::SharedPtr _base_cmd_pub;

    std::map<Gripper::Ptr, rclcpp::Publisher<JointStateSM>::SharedPtr> _gripper_cmd;

    bool _js_received;

    Eigen::VectorXd qtmp, vtmp;


};

}
#endif // ROBOTINTERFACE2_ROS_H
