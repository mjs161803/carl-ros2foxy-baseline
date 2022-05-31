#include <stdio.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "carl_interfaces/msg/arduino_command_a.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArduinoCommunicator : public rclcpp::Node {
    public:
        ArduinoCommunicator() : Node("arduino_comm") {
            battery_publisher_ = this->create_publisher<std_msgs::msg::String>("battery_voltages", 10);
            
	    battery_timer_ = this->create_wall_timer(10s, std::bind(&ArduinoCommunicator::battery_timer_callback, this));
	    
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

        void battery_timer_callback() {
            unsigned char msg[] = {'C'};
	    char read_buf [256];

	    write(this->arduino, msg, sizeof(msg));
	    int n = read(this->arduino, &read_buf, sizeof(read_buf));
	    std::string voltages = "";
	    for (int i = 0; i < (n-1); i++) {
		    voltages = voltages + read_buf[i];
	    }

	    auto message = std_msgs::msg::String();
	    message.data = voltages;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            battery_publisher_->publish(message);
        }

	void rpm_comm_callback(const carl_interfaces::msg::ArduinoCommandA::SharedPtr msg) const {
		unsigned char read_buf[256];
		
		RCLCPP_INFO(this->get_logger(), "ArduinoCommunicator received rpm command: %d %d %d %d", msg->left_rpm, msg->right_rpm, msg->left_ticks, msg->right_ticks);
		struct {
			char header = 0x41;
			short int lrpm = msg->left_rpm;
			short int rrpm = msg->right_rpm;
			unsigned long lticks = msg->left_ticks;
			unsigned long rticks = msg->right_ticks;
		} ard_cmd_a;

		RCLCPP_INFO(this->get_logger(), "Size of command to Arduino: %d", sizeof(ard_cmd_a));
		
		//auto *msg_ptr = &ard_cmd_a;

		//write(this->arduino, msg_ptr, sizeof(ard_cmd_a));
		//int n = read(this->arduino, &read_buf, sizeof(read_buf));

	    	//std::string ard_response = "Arduino Response: ";
	    	//for (int i = 0; i < (n-1); i++) {
		//	ard_response = ard_response + read_buf[i];
	    	//}

		//std::cout << ard_response << std::endl;
	}

    	rclcpp::TimerBase::SharedPtr battery_timer_;
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_publisher_;
	rclcpp::Subscription<carl_interfaces::msg::ArduinoCommandA>::SharedPtr rpm_command_subscription_;
	int serial_port;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommunicator>());
    rclcpp::shutdown();
    return 0;
}
