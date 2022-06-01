#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "carl_interfaces/msg/arduino_command_a.hpp"

#define WHEEL_RADIUS 4.005
#define WHEEL_BASE 160.0
#define DRIVE_RPM 12000
#define TURN_RPM 7000

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotorController : public rclcpp::Node {
	public:
		MotorController() : Node("motor_controller") {
			motor_command_publisher_ = this->create_publisher<carl_interfaces::msg::ArduinoCommandA>("motor_commands", 10);
			teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("carl_teleop_commands", 10, std::bind(&MotorController::motor_command_callback, this, _1));
		}
	private: 
		void motor_command_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) const {
			// CARL acts as its own frame of reference, with +/- linear.x correlating to forward and backward.
			// +/- angular.z indicates a rotation left (-) or right (+) from current azimuth
			// Either linear.x or angular.z should be non-zero for each callback, but not both.

			double delta_x = twist_msg->linear.x; // forward or backware x centimeters
			double delta_theta = twist_msg->angular.z; // rotate left or right by z degrees

			// Calculate ticks from twist message parameters
			unsigned int num_l_ticks;
			unsigned int num_r_ticks;
			short int l_rpm;
			short int r_rpm;

			if (delta_x != 0.0) { // 	Twist command was to move forward or backward
				num_l_ticks = (unsigned int)((abs(delta_x) / (6.2832*WHEEL_RADIUS))*746.94);		
				num_r_ticks = num_l_ticks;
				l_rpm = DRIVE_RPM;
				r_rpm = DRIVE_RPM;
				if (delta_x < 0.0) {
					l_rpm *= -1;
					r_rpm *= -1;
				}
			} else { // 			Twist command was to rotate left or right
				double arc_dist = (WHEEL_BASE / 2.0)*(abs(delta_theta) * (6.2832 / 360.0));
			       	num_l_ticks = (unsigned int)((arc_dist / (6.2832*WHEEL_RADIUS))*746.94);	
				num_r_ticks = num_l_ticks;
				l_rpm = TURN_RPM;
				r_rpm = TURN_RPM;
				if (delta_theta < 0.0) {
					l_rpm *= -1;
				} else {
					r_rpm *= -1;
				}
			}

			auto message = carl_interfaces::msg::ArduinoCommandA();
			message.left_rpm = l_rpm;
			message.right_rpm = r_rpm;
			message.left_ticks = num_l_ticks;
			message.right_ticks = num_r_ticks;

			RCLCPP_INFO(this->get_logger(), "Publishing: left_rpm = %d, right_rpm =%d, left_ticks = %d, right_ticks = %d", message.left_rpm, message.right_rpm, message.left_ticks, message.right_ticks);
			motor_command_publisher_->publish(message);
		}

		rclcpp::Publisher<carl_interfaces::msg::ArduinoCommandA>::SharedPtr motor_command_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorController>());
	rclcpp::shutdown();

	return 0;
}



