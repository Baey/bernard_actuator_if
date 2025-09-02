#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "candle.hpp"
#include "bernard/config.hpp"

class ActuatorStatePublisher : public rclcpp::Node
{
public:
	ActuatorStatePublisher(std::shared_ptr<mab::Candle> candle);

private:
	void publish_joint_states();
	void publish_joint_temperatures();

	size_t count_;
	rclcpp::TimerBase::SharedPtr state_timer_;
	rclcpp::TimerBase::SharedPtr temp_timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr temp_publisher_;
	std::shared_ptr<mab::Candle> candle_;
};

class ActuatorCommandSubscriber : public rclcpp::Node
{
public:
	ActuatorCommandSubscriber(std::shared_ptr<mab::Candle> candle);

private:
	void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_subscriber_;
    std::shared_ptr<mab::Candle> candle_;
};