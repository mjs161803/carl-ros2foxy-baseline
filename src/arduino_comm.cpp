// This node will interface with CARL's Arduino Motor Controller via serial device

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ArduinoCommunicator : public rclcpp::Node {
    public:
        ArduinoCommunicator() : Node("arduino_comm") {
            publisher_ = this->create_publisher<std_msgs::msg::String>("battery_voltages", 10);
            timer_ = this->create_wall_timer(5s, std::bind(&ArduinoCommunicator::timer_callback, this));
        }

    private:
        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "10.6 volts";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommunicator>());
    rclcpp::shutdown();
    return 0;
}