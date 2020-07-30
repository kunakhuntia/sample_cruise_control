#include "cruise_control/SensorNode.hpp"
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

SensorNode::SensorNode(std::string name) : Node(name)
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                                  .history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST)
                                  .keep_last(10)
                                  .reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE)
                                  .avoid_ros_namespace_conventions(false);
    
    m_paramClient =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "/parameter_server");
    m_paramClient->wait_for_service();
    auto parameters_future = m_paramClient->get_parameters(
        {"initialspeed", "finalspeed"},
        std::bind(&SensorNode::CallbackGlobalParam, this, std::placeholders::_1));

    m_velocityPublisher = this->create_publisher<std_msgs::msg::String>(
        "velocity",
        qos_profile);

    auto period = 50ms;
    m_timer = this->create_wall_timer(period, std::bind(&SensorNode::publish, this));

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");

    m_accelerationSubscriber = this->create_subscription<std_msgs::msg::String>(
        "acceleration",
        qos_profile,
        std::bind(&SensorNode::SimpleCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber created!!");
    
    
    
}

void SensorNode::publish()
{
    auto speed = CalculateVelocity();
    std::stringstream ss;
    ss << speed;
    auto message = std_msgs::msg::String();
    message.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "Publishing Velocity: %s", message.data.c_str());
    m_velocityPublisher->publish(message);
}

void SensorNode::SimpleCallback(const std_msgs::msg::String::SharedPtr msg)
{
    m_acceleration = std::atof(msg->data.c_str());
}

void SensorNode::CallbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
{
    auto result = future.get();
    auto param = result.at(0);
    std::string temp = param.get_name();
    m_initialSpeed = std::atof(param.as_string().c_str());
    param = result.at(1);
    m_finalSpeed = std::atof(param.as_string().c_str());
}

float SensorNode::CalculateVelocity()
{
    m_initialSpeed = (m_initialSpeed * KMPERHRTOMETERPERSEC +
                      m_acceleration * TIME_OFFSET_IN_SEC) *
                    METERPERSECTOKMPERHR;

    if (m_initialSpeed >= m_finalSpeed)
    {
      m_initialSpeed = m_finalSpeed;
    }
    return m_initialSpeed;
}