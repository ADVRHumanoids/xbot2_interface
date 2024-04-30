#ifndef XBOT2IFC_CONFIG_FROM_PARAM_HPP
#define XBOT2IFC_CONFIG_FROM_PARAM_HPP

#include <ros/ros.h>
#include <xbot2_interface/logger.h>
#include <xbot2_interface/xbotinterface2.h>

namespace XBot::Utils {

inline XBot::ConfigOptions ConfigOptionsFromParamServer(ros::NodeHandle n = ros::NodeHandle())
{
    XBot::ConfigOptions ret;

    ros::NodeHandle npr("~");

    std::string urdf_param, srdf_param, urdf, srdf;

    if(!n.searchParam("robot_description", urdf_param))
    {
        throw std::out_of_range(
            "could not find parameter 'robot_description' within ns '" +
            n.getNamespace() + "'");
    }

    Logger::info("found urdf at %s \n", urdf_param.c_str());

    n.getParam(urdf_param, urdf);

    if(!ret.set_urdf(urdf))
    {
        throw std::invalid_argument(
            "unable to parse urdf at " + urdf_param
            );
    }

    if(n.searchParam("robot_description_semantic", srdf_param))
    {
        Logger::info("found srdf at %s \n", srdf_param.c_str());

        n.getParam(srdf_param, srdf);

        if(!ret.set_srdf(srdf))
        {
            Logger::error("unable to parse urdf at %s \n", srdf_param.c_str());
        }
    }

    // set robot type
    ret.set_parameter<std::string>("robot_type", "ros");

    // set model type from private parameters
    std::string model_type;
    if(npr.getParam("model_type", model_type))
    {
        ret.set_parameter("model_type", model_type);
    }

    return ret;
}


}

#endif // CONFIG_FROM_PARAM_HPP
