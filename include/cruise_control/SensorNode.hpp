#ifndef SENSOR_NODE_HPP
#define SENSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include<memory>


class SensorNode : public rclcpp::Node {

public:

    SensorNode(std::string name = "SensorNode");
    ~SensorNode()=default;
    void publish();   
private:
    /// Handle to publish velocity
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_velocityPublisher;
    /// Handle to subscribe acceleration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_accelerationSubscriber;
    /// Callback for subsciber
    void SimpleCallback(const std_msgs::msg::String::SharedPtr msg);
    //static void setFinalVelocity();
    /// Callback to access config params from paramserver
    void CallbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future);
    /// Helper to calculate velocity
    float CalculateVelocity();
    /// Timer for publising velocity (50ms)
    rclcpp::TimerBase::SharedPtr m_timer;
    /// Handle to access parameters from paramserver
    std::shared_ptr<rclcpp::AsyncParametersClient> m_paramClient;
    float m_initialSpeed;
    float m_finalSpeed;
    float m_acceleration = 0.0f;
    static constexpr float KMPERHRTOMETERPERSEC = 5.0f/18.0f;
    static constexpr float TIME_OFFSET_IN_SEC = 0.05f;
    static constexpr float METERPERSECTOKMPERHR = 18.0f/5.0f;
    
};

#endif // SENSOR_NODE_HPP