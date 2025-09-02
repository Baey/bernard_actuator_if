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

	// auto candle = std::make_shared<mab::Candle>(mab::CAN_BAUD_8M, true);
	// auto ids = candle->ping();
	// for (auto &id : ids)
	// {
	// 	candle->addMd80(id);
	// }

	// auto state_pub = std::make_shared<ActuatorStatePublisher>(candle);
	// auto cmd_sub = std::make_shared<ActuatorCommandSubscriber>(candle);

	// candle->begin();

	// rclcpp::executors::MultiThreadedExecutor exec;
	// exec.add_node(state_pub);
	// exec.add_node(cmd_sub);
	// exec.spin();

	// candle->end();
	rclcpp::shutdown();

	return 0;
}