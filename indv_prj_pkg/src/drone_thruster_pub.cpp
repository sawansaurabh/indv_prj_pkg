#include <chrono>
#include <memory>
#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "interfaces_indv_prj_pkg/msg/thruster_forces.hpp"

#include <termios.h>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<interfaces_indv_prj_pkg::msg::ThrusterForces>("thruster_forces_to_unity", 10);

    setNonCanonicalMode(true);

    timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::readKeyPresses, this));
  }

  ~MinimalPublisher()
  {
    setNonCanonicalMode(false);
  }

private:
  void setNonCanonicalMode(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
  }

  bool kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
  }

  void readKeyPresses()
  {
    auto message = interfaces_indv_prj_pkg::msg::ThrusterForces();
    if (kbhit()) {
      char buffer[1024];
      int bytesRead = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
      if (bytesRead > 0) {
        buffer[bytesRead] = '\0';

        
        if (strchr(buffer, 27)) { // 27 is the ASCII code for the Esc key
          std::cout << "Escape is pressed. Exiting..." << std::endl;
          exit(0);
        }
        if (strchr(buffer, 'w')) {
          message.thrusters[0] = 1;
          message.forces[0] = 5;
        }
        if (strchr(buffer, 's')) {
          message.thrusters[1] = 1;
          message.forces[1] = 5;
        }
        if (strchr(buffer, 'a')) {
          message.thrusters[2] = 1;
          message.forces[2] = 5;
        }
        if (strchr(buffer, 'd')) {
          message.thrusters[3] = 1;
          message.forces[3] = 5;
        }

        // RCLCPP_INFO(this->get_logger(), "You pressed: %c", message.data);
      }
    }
    publisher_->publish(message);
  }

  struct termios oldt_, newt_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces_indv_prj_pkg::msg::ThrusterForces>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  std::cout << "Exiting..." << std::endl;
  rclcpp::shutdown();
  return 0;
}
