#include "rclcpp/rclcpp.hpp"

#include "cruise_control/AlgorithmNode.hpp"
#include "cruise_control/ParamServer.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::shared_ptr<AlgorithmNode> node = std::make_shared<AlgorithmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


