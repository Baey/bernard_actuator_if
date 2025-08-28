#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "bernard_control/joint_config.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ActuatorStatePublisher : public rclcpp::Node
{
public:
	ActuatorStatePublisher()
			: Node("actuator_state_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

		// Candle initialization
		candle_ = std::make_unique<mab::Candle>(mab::CAN_BAUD_8M, true);
		auto ids = candle_->ping();
		if (ids.size() == 0)
		{
			RCLCPP_ERROR(this->get_logger(), "No actuators found on CAN bus!");
			throw std::runtime_error("No actuators found on CAN bus!");
		}
		for (auto &id : ids)
		{
			RCLCPP_INFO(this->get_logger(), "Found drive ID: %d", id);
			candle_->addMd80(id);
		}
		candle_->begin();

		timer_ = this->create_wall_timer(
				100ms, std::bind(&ActuatorStatePublisher::publish_joint_states, this));
	}

	~ActuatorStatePublisher()
	{
		candle_->end();
	}

private:
	void publish_joint_states()
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
			msg.name.push_back(JOINT_MAP.at(md80.getId()));
			msg.position.push_back(md80.getPosition());
			msg.velocity.push_back(md80.getVelocity());
			msg.effort.push_back(md80.getTorque());
		}

		publisher_->publish(msg);
	}

	size_t count_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
	std::unique_ptr<mab::Candle> candle_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ActuatorStatePublisher>());
	rclcpp::shutdown();
	return 0;
}