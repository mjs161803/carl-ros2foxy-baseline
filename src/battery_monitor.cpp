#include <chrono>
#include <string>
#include <functional>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "carl_interfaces/srv/query_batt_voltages.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BatteryCalculator {
    public:
		BatteryCalculator() {
			min_voltage = 9.6;
			sta_duration = 600.0;
			lta_duration = 6000.0;
			sta_remaining = 0; 
			lta_remaining = 0;
			calc_start_time = std::chrono::system_clock::now();
		}
		BatteryCalculator(const float min_v, const unsigned long sta_window, const unsigned long lta_window) {
        	min_voltage = min_v;
            sta_duration = sta_window;
            lta_duration = lta_window;
            calc_start_time = std::chrono::system_clock::now();
        }
		void update_log(float, std::chrono::time_point<std::chrono::system_clock>);
		unsigned long get_sta();
        unsigned long get_lta();

    private:
		float min_voltage;      // minimum voltage before needing recharge
        unsigned long sta_duration;     // short term average window duration, in seconds
        unsigned long lta_duration;     // long term average window duration, in seconds
        std::chrono::time_point<std::chrono::system_clock> calc_start_time;     // start time when calculator object was created
        std::vector<std::pair<float, std::chrono::time_point<std::chrono::system_clock>>> measurement_log;      // vector containing std::pair's with voltage, timestamp
        unsigned long sta_remaining;    // in seconds. = NULL upon initialization, and stays NULL until enough voltage sameples are stored
        unsigned long lta_remaining;    // in seconds. = NULL upon initialization, and stays NULL until enough voltage sameples are stored

        unsigned long calc_sta();
        unsigned long calc_lta();
};

void BatteryCalculator::update_log(float voltage_measurement, std::chrono::time_point<std::chrono::system_clock> measurement_time) {
	this->measurement_log.push_back(std::make_pair(voltage_measurement, measurement_time));
	std::cout << "Updated measurement log with (" << voltage_measurement << ", )" << std::endl; 
}

class BatteryMonitor : public rclcpp::Node {
	public:
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
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr b2_lta_publisher_;
		void query_timer_callback();
};

void BatteryMonitor::query_timer_callback() {
	auto request = std::make_shared<carl_interfaces::srv::QueryBattVoltages::Request>();
	while (!batt_query_client_->wait_for_service(1s)) {
		if(!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting for 1 more second...");
	}

	auto result = batt_query_client_->async_send_request(request);
	auto timestamp = std::chrono::system_clock::now();
	
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery 1 Voltage: %f. Battery 2 Voltage: %f.", result.get()->b1_voltage, result.get()->b2_voltage);
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service query_batt_voltages.");
	}
		
	bc1_rpi.update_log(result.get()->b1_voltage, timestamp);
	bc2_motor.update_log(result.get()->b2_voltage, timestamp);
	std_msgs::msg::Float32 b1_rem_sta;
	std_msgs::msg::Float32 b1_rem_lta;
	std_msgs::msg::Float32 b2_rem_sta;
	std_msgs::msg::Float32 b2_rem_lta;
	b1_rem_sta.data = result.get()->b1_voltage; //bc1_rpi.calc_sta();
	b1_rem_lta.data = (result.get()->b1_voltage) * 10.0; //bc1_rpi.calc_lta();
	b2_rem_sta.data = result.get()->b2_voltage; //bc2_motor.calc_sta();
	b2_rem_lta.data = (result.get()->b2_voltage) * 10.0; //bc2_motor.calc_lta();
	b1_sta_publisher_->publish(b1_rem_sta);
	b1_lta_publisher_->publish(b1_rem_lta);
	b2_sta_publisher_->publish(b2_rem_sta);
	b2_lta_publisher_->publish(b2_rem_lta);
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BatteryMonitor>() );
	rclcpp::shutdown();

	return 0;
}




