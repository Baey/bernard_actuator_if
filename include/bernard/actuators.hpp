#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "candle.hpp"
#include "MD.hpp"

#include "bernard/config.hpp"

class ActuatorStatePublisher : public rclcpp::Node
{
public:
	ActuatorStatePublisher(mab::Candle* candle, std::vector<mab::MD> mds);

private:
	void publish_joint_states();
	void publish_joint_temperatures();

	size_t count_;
	rclcpp::TimerBase::SharedPtr state_timer_;
	rclcpp::TimerBase::SharedPtr temp_timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temp_publisher_;
	mab::Candle* candle_;
	std::vector<mab::MD> mds_;
};

class ActuatorCommandSubscriber : public rclcpp::Node
{
public:
	ActuatorCommandSubscriber(mab::Candle* candle, std::vector<mab::MD> mds);

private:
	void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_subscriber_;
    mab::Candle* candle_;
	std::vector<mab::MD> mds_;
};