#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "bernard/actuators.hpp"

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

ActuatorStatePublisher::ActuatorStatePublisher(mab::Candle* candle, std::vector<mab::MD> mds)
	: Node("actuator_state_publisher"), count_(0), candle_(candle), mds_(mds)
{
	state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	temp_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("driver_temperatures", 10);

	state_timer_ = this->create_wall_timer(
		10ms, std::bind(&ActuatorStatePublisher::publish_joint_states, this));
	temp_timer_ = this->create_wall_timer(
		5000ms, std::bind(&ActuatorStatePublisher::publish_joint_temperatures, this));

	RCLCPP_INFO(this->get_logger(), "ActuatorStatePublisher initialized");
}

void ActuatorStatePublisher::publish_joint_states()
{

	sensor_msgs::msg::JointState msg;
	msg.header.stamp = this->now();

	for (auto &md : mds_)
	{
		// Joints states
		msg.name.push_back(JOINT_MAP.at(md.m_canId));
		msg.position.push_back(md.getPosition().first);
		msg.velocity.push_back(md.getVelocity().first);
		msg.effort.push_back(md.getTorque().first);
	}

	state_publisher_->publish(msg);
}

void ActuatorStatePublisher::publish_joint_temperatures()
{
	mab::MDRegisters_S registerBuffer;
	std_msgs::msg::Float32MultiArray msg;

	for (auto &md : mds_)
	{
		md.readRegisters(registerBuffer.mosfetTemperature);
		msg.data.push_back(registerBuffer.mosfetTemperature.value);
	}

	temp_publisher_->publish(msg);
}

ActuatorCommandSubscriber::ActuatorCommandSubscriber(mab::Candle* candle, std::vector<mab::MD> mds)
    : Node("actuator_command_subscriber"), candle_(candle), mds_(mds)
{
    cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joint_commands", 10,
        std::bind(&ActuatorCommandSubscriber::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ActuatorCommandSubscriber initialized");
}

void ActuatorCommandSubscriber::command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

    for (size_t i = 0; i < msg->data.size(); i++)
    {
        float target = msg->data[i];
        mds_[i].setTargetTorque(target);
    }
}