#ifndef __ROBOTINTERFACEROS_CONFIGFROMPARAM_H__
#define __ROBOTINTERFACEROS_CONFIGFROMPARAM_H__

#include <xbot2_interface/xbotinterface2.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>
#include <rclcpp/wait_for_message.hpp>

using namespace std_msgs::msg;
using namespace std::chrono_literals;

namespace XBot {

inline ConfigOptions ConfigOptionsFromParams(rclcpp::Node::SharedPtr node,
                                             std::string prefix = "",
                                            std::chrono::seconds timeout = 1s)
{
    ConfigOptions opt;

    std::string urdf, srdf, jidmap;

    auto urdf_sub = node->create_subscription<String>(prefix + "robot_description",
                                                      rclcpp::QoS(1).transient_local(),
                                                      [&](String::ConstSharedPtr msg) {
                                                          urdf = msg->data;
                                                      });

    auto srdf_sub = node->create_subscription<String>(prefix + "robot_description_semantic",
                                                      rclcpp::QoS(1).transient_local(),
                                                      [&](String::ConstSharedPtr msg) {
                                                          srdf = msg->data;
                                                      });

    auto jidmap_sub = node->create_subscription<String>(prefix + "robot_description_joint_id_map",
                                                        rclcpp::QoS(1).transient_local(),
                                                        [&](String::ConstSharedPtr msg) {
                                                            jidmap = msg->data;
                                                        });
    while(urdf.empty())
    {
        rclcpp::spin_all(node, 1s);
        sleep(1);
        RCLCPP_INFO(node->get_logger(), "waiting for urdf on topic '%s'...", urdf_sub->get_topic_name());
    }

    opt.set_urdf(urdf);

    while(srdf.empty())
    {
        rclcpp::spin_all(node, 1s);
        sleep(1);
        RCLCPP_INFO(node->get_logger(), "waiting for srdf on topic '%s'...", srdf_sub->get_topic_name());
    }

    opt.set_srdf(srdf);

    opt.set_parameter("model_type", node->get_parameter_or<std::string>("model_type", "pin"));

    opt.set_parameter<std::string>("framework", "ROS2");

    return opt;
}

} // namespace XBot

#endif
