#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class ClockPublisher : public rclcpp::Node
{
public:
    ClockPublisher() : Node("clock_publisher")
    {
        publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("clock", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&ClockPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = builtin_interfaces::msg::Time();
        msg = this->now();
        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing current time: '%ld.%09ld'", msg.data.sec, msg.data.nanosec);
    }

    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClockPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

