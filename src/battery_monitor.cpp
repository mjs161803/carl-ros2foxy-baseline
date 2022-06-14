#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "carl_interfaces/srv/query_batt_voltages.hpp"
#include "battery_calculator.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BatteryMonitor : public rclcpp::Node {
	public:
		query_voltages_client_; 
		BatteryMonitor() : Node("battery_monitor") {
			query_timer_ = this->create_wall_timer(30s, std::bind(&BatteryMonitor::query_timer_callback, this));
			batt_query_client_ = this->create_client<carl_interfaces::srv::QueryBattVoltages>("query_batt_voltages");
			b1_sta_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery1_rem_sta", 1);
			b1_lta_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery1_rem_lta", 1);
			b2_lta_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery2_rem_lta", 1);
			b2_lta_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery2_rem_lta", 1);
		}
	private:
		BatteryCalculator bc1_rpi, bc2_motor;
		rclcpp::TimerBase::SharedPtr query_timer_;
		rclcpp::Client<carl_interfaces::srv::QueryBattVoltages>::SharedPtr batt_query_client_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr b1_sta_publisher_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr b1_lta_publisher_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr b2_sta_publisher_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr b2_sta_publisher_;
		void query_timer_callback();
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BatteryMonitor>() );
	rclcpp::shutdown();

	return 0;
}


void BatteryMonitor::query_timer_callback() {
	auto request = std::make_shared<carl_interfaces::srv::QueryBattVoltages::Request>();
	while (!batt_query_client_->wait_for_service(1s)) {
		if(!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting for 1 more second...");
	}

	auto result = batt_query_client_->async_send_request(request);
	if (rclcpp::spin_until_future_complete(this, result) == rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery 1 Voltage: %f. Battery 2 Voltage: %f", result.get()->b1_voltage, result.get()->b2_voltage);
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service query_batt_voltages.");
	}
	
	auto timestamp = std::chrono::system_clock::now();
	bc1_rpi.update_log(result.get()->b1_voltage, timestamp);
	bc2_motor.update_log(result.get()->b2_voltage, timestamp);
	std_msgs::msg::Float32 b1_rem_sta;
	std_msgs::msg::Float32 b1_rem_lta;
	std_msgs::msg::Float32 b2_rem_sta;
	std_msgs::msg::Float32 b2_rem_lta;
	b1_rem_sta.data = bc1_rpi.get_sta();
	b1_rem_lta.data = bc1_rpi.get_lta();
	b2_rem_sta.data = bc2_motor.get_sta();
	b2_rem_lta.data = bc2_motor.get_lta();
	b1_sta_publisher_->publish(b1_rem_sta);
	b1_lta_publisher_->publish(b1_rem_lta);
	b2_sta_publisher_->publish(b2_rem_sta);
	b2_lta_publisher_->publish(b2_rem_lta);
}

