#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include<memory>

class UserInput : public rclcpp::Node
{
public:
    explicit UserInput(std::string name="UserInput");
    ~UserInput()=default;
    void PublishChangedVelocity(std_msgs::msg::String velocity);
    void CreatePublishers();
    void WaitForUserInput();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};