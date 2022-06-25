#include <chrono>
#include <string>
#include <functional>
#include <utility>
#include <deque>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "carl_interfaces/srv/query_batt_voltages.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BatteryCalculator {
    public:
	    BatteryCalculator() {
		    battery_remaining = 100;
	    }
	    void update_log(float, std::chrono::time_point<std::chrono::system_clock>);
	    int calc_rem();
    private:
	    std::deque<std::pair<float, std::chrono::time_point<std::chrono::system_clock>>> measurement_log;      // deque containing std::pair's with voltage, timestamp
	    int battery_remaining;    // % of battery remaining
	    std::array<std::pair<float, float>, 11> discharge_curve {std::make_pair(0.0, 0.0), 
		    std::make_pair(10.0, 9.2), 
		    std::make_pair(20.0, 9.5), 
		    std::make_pair(30.0, 9.65), 
		    std::make_pair(40.0, 9.8), 
		    std::make_pair(50.0, 9.89), 
		    std::make_pair(60.0, 9.91), 
		    std::make_pair(70.0, 9.95), 
		    std::make_pair(80.0, 10.0), 
		    std::make_pair(90.0, 10.1), 
		    std::make_pair(100, 11.1)};
	    int lookup_voltage(const float);
};

void BatteryCalculator::update_log(const float voltage_measurement, const std::chrono::time_point<std::chrono::system_clock> measurement_time) {
	this->measurement_log.push_back(std::make_pair(voltage_measurement, measurement_time));
	std::time_t now_c = std::chrono::system_clock::to_time_t(measurement_time);
	std::cout << "Updated measurement log with (" << voltage_measurement << ", [" << now_c << "]). # Measurements in Log: " << this->measurement_log.size() << std::endl; 
	if (this->measurement_log.size() > 4) {
		this->measurement_log.pop_front();
	}
}

int BatteryCalculator::calc_rem() {
	if (this->measurement_log.size() == 0) {
		return 0;
	}

	float avg_voltage {0.0};
	for (auto &i : this->measurement_log) {
		avg_voltage += i.first;
	}
	avg_voltage = avg_voltage / float(this->measurement_log.size());
	return lookup_voltage(avg_voltage);
}

int BatteryCalculator::lookup_voltage (const float v) {
	float percent_rem {0.0};
	auto it = this->discharge_curve.begin();
	float v1 = (*it).second;
	float cap1 = (*it).first;
	float v2 {0.0};
	float cap2 {0.0};
	while (it != this->discharge_curve.end()) {
		++it;
		v2 = (*it).second;
		cap2 = (*it).first;
		if ((v >= v1) && (v <= v2)) break;
		v1 = v2;
		cap1 = cap2;
	}
	if (v > v2) {
		return 100;
	}
	// v should now be between v1 and v2
	float slope = 10.0 / (v2 - v1);
	float delta_x = v - v1;
	percent_rem = cap1 + (slope * delta_x);
	return int(percent_rem);
}

class BatteryMonitor : public rclcpp::Node {
	public:
		BatteryMonitor() : Node("battery_monitor") {
			query_timer_ = this->create_wall_timer(5s, std::bind(&BatteryMonitor::query_timer_callback, this));
			batt_query_client_ = this->create_client<carl_interfaces::srv::QueryBattVoltages>("query_batt_voltages");
			b1_remaining_publisher_ = this->create_publisher<std_msgs::msg::Int32>("battery1_rem", 1);
			b2_remaining_publisher_ = this->create_publisher<std_msgs::msg::Int32>("battery2_rem", 1);
		}
	private:
		float m_voltage = 9.6;
		BatteryCalculator bc1_rpi;
	        BatteryCalculator bc2_motor;
		rclcpp::TimerBase::SharedPtr query_timer_;
		rclcpp::Client<carl_interfaces::srv::QueryBattVoltages>::SharedPtr batt_query_client_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr b1_remaining_publisher_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr b2_remaining_publisher_;
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

	using ServiceResponseFuture = rclcpp::Client<carl_interfaces::srv::QueryBattVoltages>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
		auto result = future.get();
		auto timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
		if ((result.get()->b1_voltage != 0.0) && (result.get()->b2_voltage != 0.0)) {
			std::time_t now_c = std::chrono::system_clock::to_time_t(timestamp);
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[@timestamp %lu] Battery 1 Voltage: %f. Battery 2 Voltage: %f.", now_c, result.get()->b1_voltage, result.get()->b2_voltage);
			bc1_rpi.update_log(result.get()->b1_voltage, timestamp);
			bc2_motor.update_log(result.get()->b2_voltage, timestamp);
		}
	};

	auto future_result = batt_query_client_->async_send_request(request, response_received_callback);
	
	std_msgs::msg::Int32 b1_rem;
	std_msgs::msg::Int32 b2_rem;
	b1_rem.data = bc1_rpi.calc_rem();
	b2_rem.data = bc2_motor.calc_rem();
	if (b1_rem.data) {
		b1_remaining_publisher_->publish(b1_rem);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing Battery 1 Capacity Remaining: %d.", b1_rem.data);
	}
	if (b2_rem.data) {
		b2_remaining_publisher_->publish(b2_rem);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing Battery 2 Capacity Remaining: %d.", b2_rem.data);
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BatteryMonitor>() );
	rclcpp::shutdown();

	return 0;
}




