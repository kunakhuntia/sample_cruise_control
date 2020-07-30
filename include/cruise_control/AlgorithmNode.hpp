#ifndef ALGORITHM_NODE_HPP
#define ALGORITHM_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class AlgorithmNode : public rclcpp::Node 
{

 public:
    AlgorithmNode(std::string name = "AlgorithmNode");
    ~AlgorithmNode()=default;
    
 private:
    /// Callback to process the subscribe velocity
    void SimpleCallback(const std_msgs::msg::String::SharedPtr msg);
    /// Helper to to calculate the acceleration
    float CalculateAccleration(float speed);
    /// Helper to process the subscribe data
    void ProcessData(std::string data);
    /// Callback to access parameters from paramserver node
    void CallbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future);
    /// Callback to set updated velocity on subscription
    void VelocityChangeCallback(const std_msgs::msg::String::SharedPtr msg);
    static constexpr float KMPERHRTOMETERPERSEC = 5.0f/18.0f;
    static constexpr float METERPERSECTOKMPERHR = 18.0f/5.0f;
    static constexpr float TIME_OFFSET_IN_SEC = 0.05f;
    /// Previous velocity
    float m_initialSpeed = 0.0f;
    /// Starting speed
    float m_startSpeed;
    /// Changed speed
    volatile float m_finalSpeedChanged;
    /// Handle to subscriber for velocity
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_velocitySubscriber;
    /// Handle to publisher for acceleration
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_accelerationPublisher;
    /// Handle tp param client 
    std::shared_ptr<rclcpp::AsyncParametersClient> m_paramClient;
    /// Handle to subscriber for velocity change
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_changedVelocitySubscriber;
    float m_acceleration=0.0f;
};

#endif // ALGORITHM_NODE_HPP