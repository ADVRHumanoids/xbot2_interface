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

using namespace xbot_msgs::msg;
using namespace std_msgs::msg;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;

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

    std::vector<rclcpp::SubscriptionBase::SharedPtr> _subs;

    rclcpp::Publisher<JointCommand>::SharedPtr _cmd_pub;

    rclcpp::Publisher<Twist>::SharedPtr _base_cmd_pub;

    bool _js_received;

    Eigen::VectorXd qtmp, vtmp;


};

}
#endif // ROBOTINTERFACE2_ROS_H
