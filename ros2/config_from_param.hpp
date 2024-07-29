#ifndef __ROBOTINTERFACEROS_CONFIGFROMPARAM_H__
#define __ROBOTINTERFACEROS_CONFIGFROMPARAM_H__

#include <xbot2_interface/xbotinterface2.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf_parser/urdf_parser.h>

using namespace std_msgs::msg;
using namespace std::chrono_literals;

namespace XBot {

inline ConfigOptions ConfigOptionsFromParams(rclcpp::Node::SharedPtr node,
                                             std::string prefix = "")
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

    while (urdf.empty()) {
        rclcpp::spin_some(node);
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "waiting for urdf on topic " << urdf_sub->get_topic_name());
        node->get_clock()->sleep_for(100ms);
    }

    while (srdf.empty()) {
        rclcpp::spin_some(node);
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "waiting for srdf on topic " << srdf_sub->get_topic_name());
        node->get_clock()->sleep_for(100ms);
    }

    opt.set_urdf(urdf);
    opt.set_srdf(srdf);


    opt.set_parameter("model_type", node->get_parameter_or<std::string>("model_type", "RBDL"));

    opt.set_parameter<std::string>("framework", "ROS2");

    return opt;
}

} // namespace XBot

#endif
