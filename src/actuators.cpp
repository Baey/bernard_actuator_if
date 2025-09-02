#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "bernard/actuators.hpp"

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

ActuatorStatePublisher::ActuatorStatePublisher(std::shared_ptr<mab::Candle> candle)
	: Node("actuator_state_publisher"), count_(0), candle_(candle)
{
	state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	temp_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("joint_temperatures", 10);

	state_timer_ = this->create_wall_timer(
		10ms, std::bind(&ActuatorStatePublisher::publish_joint_states, this));
	temp_timer_ = this->create_wall_timer(
		5000ms, std::bind(&ActuatorStatePublisher::publish_joint_temperatures, this));
}

void ActuatorStatePublisher::publish_joint_states()
{
	if (candle_->md80s.empty())
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
							 "No actuators detected.");
		return;
	}

	sensor_msgs::msg::JointState msg;
	msg.header.stamp = this->now();

	for (auto &md80 : candle_->md80s)
	{
		// Joints states
		msg.name.push_back(JOINT_MAP.at(md80.getId()));
		msg.position.push_back(md80.getPosition());
		msg.velocity.push_back(md80.getVelocity());
		msg.effort.push_back(md80.getTorque());
	}

	state_publisher_->publish(msg);
}

void ActuatorStatePublisher::publish_joint_temperatures()
{
	if (candle_->md80s.empty())
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
							 "No actuators detected.");
		return;
	}

	std_msgs::msg::UInt8MultiArray msg;

	for (auto &md80 : candle_->md80s)
	{
		msg.data.push_back(md80.getTemperature());
	}

	temp_publisher_->publish(msg);
}

ActuatorCommandSubscriber::ActuatorCommandSubscriber(std::shared_ptr<mab::Candle> candle)
    : Node("actuator_command_subscriber"), candle_(candle)
{
    cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joint_commands", 10,
        std::bind(&ActuatorCommandSubscriber::command_callback, this, std::placeholders::_1));
    
    for (auto &md : candle_->md80s)
    {
        candle_->controlMd80Mode(md, mab::Md80Mode_E::RAW_TORQUE);
    }

    RCLCPP_INFO(this->get_logger(), "ActuatorCommandSubscriber initialized");
}

void ActuatorCommandSubscriber::command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (candle_->md80s.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "No actuators detected.");
        return;
    }

    if (msg->data.size() != candle_->md80s.size())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Received %zu commands, but bernard has %zu actuators!",
                     msg->data.size(), candle_->md80s.size());
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i++)
    {
        float target = msg->data[i];
        candle_->md80s[i].setTargetTorque(target);
    }
}