#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "candle.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bernard/actuators.hpp"


int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto candle = std::make_shared<mab::Candle>(mab::CAN_BAUD_8M, true);
	auto ids = candle->ping();
	for (auto &id : ids)
	{
		candle->addMd80(id);
		RCLCPP_INFO(rclcpp::get_logger("actuator_state_publisher"), "Found drive %s, (ID: %d)", JOINT_MAP.at(id).c_str(), id);
	}

	candle->begin();

	auto state_pub = std::make_shared<ActuatorStatePublisher>(candle);

	rclcpp::spin(state_pub);

	candle->end();
	rclcpp::shutdown();

	return 0;
}