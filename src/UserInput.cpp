#include "cruise_control/UserInput.hpp"

#include <iostream>
#include <utility>
#include <signal.h>

UserInput::UserInput(std::string name) : Node(name)
{
    CreatePublishers();
    WaitForUserInput();
}

void UserInput::CreatePublishers()
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                                  .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                                  .keep_last(10)
                                  .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
                                  .avoid_ros_namespace_conventions(false);

    pub_ = create_publisher<std_msgs::msg::String>("changefinalvelocity", qos_profile);
}

void UserInput::WaitForUserInput()
{
    std::string velocity;
    std::cout << "Please enter the new velocity:";
    while (1)
    {
        std::cin >> velocity;
        std::cout << std::endl;
        if (!velocity.empty())
        {
            std_msgs::msg::String msg;
            msg.data = velocity;
            std::cout << "New Velocity to be published: " << velocity << std::endl;
            PublishChangedVelocity(msg);
            std::cout << "Velocity has been changed to: " << velocity << std::endl;
            break;
        }
        else
        {
            std::ignore = velocity;
            std::cout << "Please enter a valid value";
        }
    }
}

void UserInput::PublishChangedVelocity(std_msgs::msg::String velocity)
{
    RCLCPP_INFO(this->get_logger(), "Publishing User changed Velocity: %s", velocity.data.c_str());
    pub_->publish(std::move(velocity));
}

int main(int argc, char *argv[])
{
    signal(SIGINT, SIG_DFL);
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserInput>("UserInput");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}