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
	size_t i = 0;
	while (i < ids.size())
	{
		auto id = ids[i];
		RCLCPP_INFO(rclcpp::get_logger("actuator_state_publisher"), "Adding drive %s, (ID: %d)", JOINT_MAP.at(id).c_str(), id);
		bool success;
		success = candle->addMd80(id);
		if (!success) {
			RCLCPP_ERROR(rclcpp::get_logger("actuator_state_publisher"), "Failed to add drive. Retrying...");
		} else {
			i++;
		}
	}

	auto state_pub = std::make_shared<ActuatorStatePublisher>(candle);

	candle->begin();

	rclcpp::spin(state_pub);

	candle->end();
	rclcpp::shutdown();

	return 0;
}