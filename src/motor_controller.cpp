#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "carl_interfaces/msg/arduino_command_a.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotorController : public rclcpp::Node {
	public:
		MotorController() : Node("motor_controller") {
			motor_command_publisher_ = this->create_publisher<carl_interfaces::msg::ArduinoCommandA>("motor_commands", 10);
			motor_command_timer_ = this->create_wall_timer(10s, std::bind(&MotorController::motor_command_callback, this));
		
		}
	private: 
		void motor_command_callback() {
			short int l_rpm {10000};
			short int r_rpm {-10000};
			unsigned int num_l_ticks {747};
			unsigned int num_r_ticks {747};

			auto message = carl_interfaces::msg::ArduinoCommandA();
			message.left_rpm = l_rpm;
			message.right_rpm = r_rpm;
			message.left_ticks = num_l_ticks;
			message.right_ticks = num_r_ticks;

			RCLCPP_INFO(this->get_logger(), "Publishing: left_rpm = %d, right_rpm =%d, left_ticks = %d, right_ticks = %d", message.left_rpm, message.right_rpm, message.left_ticks, message.right_ticks);
			motor_command_publisher_->publish(message);
		}

		rclcpp::TimerBase::SharedPtr motor_command_timer_;
		rclcpp::Publisher<carl_interfaces::msg::ArduinoCommandA>::SharedPtr motor_command_publisher_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorController>());
	rclcpp::shutdown();

	return 0;
}



