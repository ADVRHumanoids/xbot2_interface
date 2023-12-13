#ifndef ROBOTINTERFACE2_ROS_H
#define ROBOTINTERFACE2_ROS_H

#include <xbot2_interface/robotinterface2.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <xbot_msgs/JointState.h>
#include <xbot_msgs/JointCommand.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

namespace XBot {

class RobotInterface2Ros : public RobotInterface
{

public:

    RobotInterface2Ros(std::unique_ptr<XBotInterface> model);

    bool sense() override;

    bool move() override;

private:

    struct RosInit
    {
        RosInit();
    };

    void on_js_recv(xbot_msgs::JointStateConstPtr msg);

    RosInit _ros_init;

    ros::CallbackQueue _cbq;

    ros::NodeHandle _nh;

    ros::Subscriber _js_sub;

    ros::Publisher _cmd_pub;

    ros::Publisher _base_cmd_pub;

    bool _js_received;

    Eigen::VectorXd qtmp, vtmp;


};

}
#endif // ROBOTINTERFACE2_ROS_H
