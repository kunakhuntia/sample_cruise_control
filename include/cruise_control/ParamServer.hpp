#ifndef PARAM_SERVER_HPP
#define PARAM_SERVER_HPP

#include "rclcpp/rclcpp.hpp"

class ParamServer : public rclcpp::Node 
{
public:
    ParamServer(std::string name, rclcpp::NodeOptions options);
};
#endif // PARAM_SERVER_HPP