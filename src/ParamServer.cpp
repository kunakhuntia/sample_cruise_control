#include "cruise_control/ParamServer.hpp"

ParamServer::ParamServer(std::string name, rclcpp::NodeOptions options) : Node(name, options)
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ParamServer>("parameter_server", options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}