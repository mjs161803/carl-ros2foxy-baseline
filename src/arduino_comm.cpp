#include <stdio.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "carl_interfaces/msg/arduino_command_a.hpp"
#include "carl_interfaces/srv/query_batt_voltages.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArduinoCommunicator : public rclcpp::Node {
    public:
        ArduinoCommunicator() : Node("arduino_comm") {
	    batt_service_ = this->create_service<carl_interfaces::srv::QueryBattVoltages>("query_batt_voltages",  std::bind(&ArduinoCommunicator::batt_query_callback, this, _1));
	    rpm_command_subscription_ = this->create_subscription<carl_interfaces::msg::ArduinoCommandA>("motor_commands", 10, std::bind(&ArduinoCommunicator::rpm_comm_callback, this, _1));
	    this->arduino = open("/dev/ttyACM0", O_RDWR);
	    if (serial_port < 0) {
		    RCLCPP_INFO(this->get_logger(), "Unable to open /dev/ttyACM0.");
	    }
	    struct termios arduino_term;
	    if(tcgetattr(arduino, &arduino_term) != 0) {
		    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	    }
	    arduino_term.c_cflag &= ~PARENB;
	    arduino_term.c_cflag &= ~CSTOPB;
	    arduino_term.c_cflag &= ~CSIZE;
	    arduino_term.c_cflag |= CS8;
	    arduino_term.c_cflag &= ~CRTSCTS;
	    arduino_term.c_cflag |= CREAD | CLOCAL;
	    arduino_term.c_lflag &= ~ICANON;
	    arduino_term.c_lflag &= ~ECHO;
	    arduino_term.c_lflag &= ~ECHOE;
	    arduino_term.c_lflag &= ~ECHONL;
	    arduino_term.c_lflag &= ~ISIG;
	    arduino_term.c_iflag &= ~(IXON | IXOFF | IXANY);
	    arduino_term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
	    arduino_term.c_oflag &= ~OPOST;
	    arduino_term.c_oflag &= ~ONLCR;
	    arduino_term.c_cc[VTIME] = 10; // 1 second
	    arduino_term.c_cc[VMIN] = 0;
	    cfsetspeed(&arduino_term, B115200);
	    if (tcsetattr(arduino, TCSANOW, &arduino_term) != 0) {
		    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	    }
        }
    private:
	int arduino {0}; // file handle for serial device
	std::array<float, 2> parse_battery_message(const std::string);
    	rclcpp::TimerBase::SharedPtr battery_timer_;
    	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery1_publisher_;
    	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery2_publisher_;
	rclcpp::Subscription<carl_interfaces::msg::ArduinoCommandA>::SharedPtr rpm_command_subscription_;
	rclcpp::Service<carl_interfaces::srv::QueryBattVoltages>::SharedPtr batt_service_; 
	int serial_port;
	void rpm_comm_callback (const carl_interfaces::msg::ArduinoCommandA::SharedPtr) const;
	void batt_query_callback (const std::shared_ptr<carl_interfaces::srv::QueryBattVoltages::Request>, std::shared_ptr<carl_interfaces::srv::QueryBattVoltages::Response>);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommunicator>());
    rclcpp::shutdown();
    return 0;
}

std::array<float, 2> ArduinoCommunicator::parse_battery_message (const std::string ard_msg) {
	std::array<float, 2> voltages_result;

	std::string delim {" "};

	int delim_pos = ard_msg.find(delim); // should always be = 1
	std::string ard_msg_substr = ard_msg.substr((delim_pos+1));
	delim_pos = ard_msg_substr.find(delim);
	std::string v1_str = ard_msg_substr.substr(0, delim_pos);
	std::string v2_str = ard_msg_substr.substr((delim_pos+1));


	voltages_result[0] = std::stof(v1_str);
	voltages_result[1] = std::stof(v2_str);

	return voltages_result;
}

void ArduinoCommunicator::batt_query_callback (const std::shared_ptr<carl_interfaces::srv::QueryBattVoltages::Request> request, 
		std::shared_ptr<carl_interfaces::srv::QueryBattVoltages::Response> response) {
	unsigned char msg[] = {'C'};
	char read_buf [256];
	std::array<float, 2> battery_voltages;
	write(this->arduino, msg, sizeof(msg));
	int n = read(this->arduino, &read_buf, sizeof(read_buf));
	std::string voltages_string = "";
	for (int i = 0; i < (n-1); i++) {
		voltages_string = voltages_string + read_buf[i];
	}
	if (voltages_string[0] == '2') {
		battery_voltages = this->parse_battery_message(voltages_string);
		auto v1_message = std_msgs::msg::Float32();
		auto v2_message = std_msgs::msg::Float32();
		v1_message.data = battery_voltages[0];
		v2_message.data = battery_voltages[1];
		
		response->b1_voltage = v1_message;
		response->b2_voltage = v2_message;

		RCLCPP_INFO(this->get_logger(), "Battery Query Service Response: Computer Battery Voltage: %.2f, Motor Battery Voltage: %.2f", v1_message.data, v2_message.data);
	} else {
		RCLCPP_INFO(this->get_logger(), "Error reading response from Arduino. Expected '2' but received: %s\n", voltages_string.c_str());
	}
}


void ArduinoCommunicator::rpm_comm_callback(const carl_interfaces::msg::ArduinoCommandA::SharedPtr msg) const {
		unsigned char read_buf[256];
		unsigned char ard_cmd_a_buff[13];
		short int local_lrpm = msg->left_rpm;
		short int local_rrpm = msg->right_rpm;
		unsigned long local_left_ticks = msg->left_ticks;
		unsigned long local_right_ticks = msg->right_ticks;
		struct cmd_struct {
			unsigned char header;
			short int lrpm;
			short int rrpm;
			unsigned int lticks;
			unsigned int rticks;
		} ard_cmd_a;
		ard_cmd_a.header = 0x41;
		ard_cmd_a.lrpm = local_lrpm;
		ard_cmd_a.rrpm = local_rrpm;
		ard_cmd_a.lticks = local_left_ticks;
		ard_cmd_a.rticks = local_right_ticks;
		memcpy((ard_cmd_a_buff+0), (char *)&ard_cmd_a.header, 1);
		memcpy((ard_cmd_a_buff+1), (char *)&ard_cmd_a.lrpm, 2);
		memcpy((ard_cmd_a_buff+3), (char *)&ard_cmd_a.rrpm, 2);
		memcpy((ard_cmd_a_buff+5), (char *)&ard_cmd_a.lticks, 4);
		memcpy((ard_cmd_a_buff+9), (char *)&ard_cmd_a.rticks, 4);
		RCLCPP_INFO(this->get_logger(), "ArduinoCommunicator received rpm command: %d %d %d %d", local_lrpm, local_rrpm, local_left_ticks, local_right_ticks);
		RCLCPP_INFO(this->get_logger(), "ard_cmd_a_buff contents: %X %X %X %X %X %X %X %X %X %X %X %X %X", 
				ard_cmd_a_buff[0], 
				ard_cmd_a_buff[1], 
				ard_cmd_a_buff[2], 
				ard_cmd_a_buff[3], 
				ard_cmd_a_buff[4], 
				ard_cmd_a_buff[5], 
				ard_cmd_a_buff[6], 
				ard_cmd_a_buff[7], 
				ard_cmd_a_buff[8], 
				ard_cmd_a_buff[9], 
				ard_cmd_a_buff[10], 
				ard_cmd_a_buff[11], 
				ard_cmd_a_buff[12]); 
		write(this->arduino, ard_cmd_a_buff, sizeof(ard_cmd_a_buff));
		int n = read(this->arduino, &read_buf, sizeof(read_buf));
	    	std::string ard_response = "Arduino Response: ";
	    	for (int i = 0; i < (n-1); i++) {
			ard_response = ard_response + (char)read_buf[i];
	    	}
		std::cout << ard_response << std::endl;
	}
