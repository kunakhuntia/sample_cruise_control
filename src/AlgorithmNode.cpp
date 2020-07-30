#include "cruise_control/AlgorithmNode.hpp"
#include <sstream>
#include <cmath>


using namespace std::chrono_literals;

AlgorithmNode::AlgorithmNode(std::string name) : Node(name)
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
            {"initialspeed", "finalspeed", "acceleration"},
            std::bind(&AlgorithmNode::CallbackGlobalParam, this, std::placeholders::_1));
    
    m_velocitySubscriber = this->create_subscription<std_msgs::msg::String>(
        "velocity",
        qos_profile,
        std::bind(&AlgorithmNode::SimpleCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber created!!");

    m_accelerationPublisher = this->create_publisher<std_msgs::msg::String>(
        "acceleration",
        qos_profile);

    m_changedVelocitySubscriber = this->create_subscription<std_msgs::msg::String>(
        "changefinalvelocity",
        qos_profile,
        std::bind(&AlgorithmNode::VelocityChangeCallback, this, std::placeholders::_1));
}

void AlgorithmNode::SimpleCallback(const std_msgs::msg::String::SharedPtr msg)
{
    ProcessData(std::move(msg->data));
}
void AlgorithmNode::ProcessData(std::string data)
{
    float speed = std::atof(data.c_str());
    auto acc = CalculateAccleration(speed);
    std::stringstream ss;
    ss << acc;
    auto message = std_msgs::msg::String();
    message.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "Publishing Accleration: %s", message.data.c_str());
    m_accelerationPublisher->publish(message);
}

float AlgorithmNode::CalculateAccleration(float speed)
{
    static bool bFlag = false;
    float newAcc =  0.0f;
    float acc = ((speed - m_initialSpeed) * KMPERHRTOMETERPERSEC) / TIME_OFFSET_IN_SEC;
    if(acc == 0 and m_startSpeed == m_initialSpeed)
    {
        acc = m_acceleration;
    }
    if(!bFlag and m_finalSpeedChanged != 0.0f and m_finalSpeedChanged < speed)
    {
        static constexpr float SPEED_CHANGE_TIME_IN_SEC = 10.0f;
        auto newAcc = (m_finalSpeedChanged - speed) * KMPERHRTOMETERPERSEC / SPEED_CHANGE_TIME_IN_SEC;
        acc = newAcc;
        bFlag = true;
    }
    else if (fabs(m_finalSpeedChanged - speed) <= 0.2)
    {
        acc = 0;
        bFlag = false;
    }
    if(bFlag)
    {
        acc = newAcc;
    }
    m_initialSpeed = speed;
    return acc;
}

void AlgorithmNode::CallbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
{
    auto result = future.get();
    auto param = result.at(0);
    m_startSpeed = std::atof(param.as_string().c_str());
    m_initialSpeed = m_startSpeed;
    RCLCPP_INFO(this->get_logger(), "m_initialSpeed: %f", m_initialSpeed);
    param = result.at(2);
    m_acceleration = std::atof(param.as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "m_acceleration: %f", m_acceleration);
}

void AlgorithmNode::VelocityChangeCallback(const std_msgs::msg::String::SharedPtr msg)
{
    auto changedVelocity = atof(msg->data.c_str());
    if(changedVelocity > m_initialSpeed)
    {
        m_finalSpeedChanged = changedVelocity;
    }
}
