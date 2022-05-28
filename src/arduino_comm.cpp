#include <stdio.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class ArduinoCommunicator : public rclcpp::Node {
    public:
        ArduinoCommunicator() : Node("arduino_comm") {
            publisher_ = this->create_publisher<std_msgs::msg::String>("battery_voltages", 10);
            timer_ = this->create_wall_timer(5s, std::bind(&ArduinoCommunicator::timer_callback, this));
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

        void timer_callback() {
            unsigned char msg[] = {'C'};
	    char read_buf [256];

	    write(this->arduino, msg, sizeof(msg));
	    int n = read(this->arduino, &read_buf, sizeof(read_buf));
	    std::string voltages = "";
	    for (int i = 0; i < n; i++) {
		    voltages = voltages + read_buf[i];
	    }

	    auto message = std_msgs::msg::String();
	    message.data = voltages;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

    	rclcpp::TimerBase::SharedPtr timer_;
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	int serial_port;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommunicator>());
    rclcpp::shutdown();
    return 0;
}
