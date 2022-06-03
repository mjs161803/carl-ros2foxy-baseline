#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader {
	public:
		KeyboardReader() : kfd(0) {
			// get the console in raw mode
			tcgetattr(kfd, &cooked);
			struct termios raw;
			memcpy(&raw, &cooked, sizeof(struct termios));
			raw.c_lflag &=~ (ICANON | ECHO);
			raw.c_cc[VEOL] = 1;
			raw.c_cc[VEOF] = 2;
			tcsetattr(kfd, TCSANOW, &raw);
		}

		void readOne(char * c) {
			int rc = read(kfd, c, 1);
			if (rc < 0) {
				throw std::runtime_error("read failure");
			}
		} // end of readOne

		void shutdown() {
			tcsetattr(kfd, TCSANOW, &cooked);
		}

	private:
		int kfd;
		struct termios cooked;
};

class TeleopCarl {
	public:
		TeleopCarl();
		int keyLoop();

	private:
		void spin();
		rclcpp::Node::SharedPtr nh_;
		double linear_, angular_, l_scale_, a_scale_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

TeleopCarl::TeleopCarl(): linear_(0), angular_(0), l_scale_(10.0), a_scale_(10.0) {
	nh_ = rclcpp::Node::make_shared("teleop_carl");
	nh_->declare_parameter("scale_angular", rclcpp::ParameterValue(10.0));
	nh_->declare_parameter("scale_linear", rclcpp::ParameterValue(10.0));
	nh_->get_parameter("scale_angular", a_scale_);
	nh_->get_parameter("scale_linear", l_scale_);

	twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("carl_teleop_commands", 1);
}

KeyboardReader input;

void quit(int sig) {
	(void)sig;
	input.shutdown();
	rclcpp::shutdown();
	exit(0);
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	TeleopCarl teleop_carl;

	signal(SIGINT, quit);

	int rc = teleop_carl.keyLoop();
	input.shutdown();
	rclcpp::shutdown();

	return rc;
}

void TeleopCarl::spin() {
	while (rclcpp::ok()) {
		rclcpp::spin_some(nh_);
	}
}

int TeleopCarl::keyLoop() {
	char c;
	bool dirty = false;

	std::thread{std::bind(&TeleopCarl::spin, this)}.detach();

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the CARL robot.");
	puts("'Q' to quit.");

	for (;;) {
		try {
			input.readOne(&c);
		}
		catch (const std::runtime_error &) {
			perror("read():");
			return -1;
		}

		linear_ = angular_ = 0;
		RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);
		
		switch(c) {
			case KEYCODE_LEFT: 
				RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
				angular_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_RIGHT:
				RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_UP:
				RCLCPP_DEBUG(nh_->get_logger(), "UP");
				linear_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_DOWN:
				RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
				linear_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_Q:
				RCLCPP_DEBUG(nh_->get_logger(), "quit");
				return 0;
		}

		geometry_msgs::msg::Twist twist;

		twist.angular.z = a_scale_*angular_;
		twist.linear.x = l_scale_*linear_;
		if (dirty == true) {
			twist_pub_->publish(twist);
			dirty = false;
		}
	}

	return 0;
}




