#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Structure to hold the state of the vehicle
struct State {
    double x;  // Position x
    double y;  // Position y
    double theta;  // Orientation in radians
};

// Function to update the state of the vehicle based on the bicycle model
State updateState(const State &currentState, double speed, double steeringAngle, double wheelbase, double dt) {
    State newState;
    newState.x = currentState.x + speed * std::cos(currentState.theta) * dt;
    newState.y = currentState.y + speed * std::sin(currentState.theta) * dt;
    newState.theta = currentState.theta + (speed / wheelbase) * std::tan(steeringAngle) * dt;
    return newState;
}

class PathGeneratorNode : public rclcpp::Node {
public:
    PathGeneratorNode()
        : Node("path_generator_node"), speed_(10.0), steering_angle_(0.0), wheelbase_(2.5), dt_(0.1), num_points_(100) {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PathGeneratorNode::generatePath, this));

        // Set terminal to non-blocking mode
        setNonBlockingInput();
    }

    void getInput() {
        char ch;
        while (rclcpp::ok()) {
            fd_set set;
            struct timeval timeout;
            int rv;

            FD_ZERO(&set);        /* clear the set */
            FD_SET(STDIN_FILENO, &set); /* add our file descriptor to the set */

            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100 ms

            rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
            if (rv == -1) {
                perror("select"); // an error occurred
            } else if (rv == 0) {
                // No input within 100 ms
            } else {
                if (read(STDIN_FILENO, &ch, 1) > 0) {
                    switch (ch) {
                        case 'w':  // Increase speed
                            speed_ += 0.1;
                            break;
                        case 's':  // Decrease speed
                            speed_ -= 0.1;
                            break;
                        case 'a':  // Increase steering angle
                            steering_angle_ += 0.01;
                            break;
                        case 'd':  // Decrease steering angle
                            steering_angle_ -= 0.01;
                            break;
                        default:
                            break;
                    }
                }
            }
            std::cout << "\rSpeed: " << speed_ << " m/s, Steering Angle: " << steering_angle_ << " rad" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void generatePath() {
        std::vector<State> path;
        State currentState = {0.0, 0.0, 0.0};  // Initial state (x, y, theta)
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (int i = 0; i < num_points_; ++i) {
            path.push_back(currentState);
            currentState = updateState(currentState, speed_, steering_angle_, wheelbase_, dt_);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = path_msg.header.stamp;
            pose_stamped.header.frame_id = path_msg.header.frame_id;
            pose_stamped.pose.position.x = currentState.x;
            pose_stamped.pose.position.y = currentState.y;
            pose_stamped.pose.orientation = toQuaternion(currentState.theta);
            path_msg.poses.push_back(pose_stamped);
        }

        publisher_->publish(path_msg);
    }

    geometry_msgs::msg::Quaternion toQuaternion(double theta) {
        geometry_msgs::msg::Quaternion q;
        q.z = sin(theta / 2.0);
        q.w = cos(theta / 2.0);
        return q;
    }

    void setNonBlockingInput() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double speed_;
    double steering_angle_;
    double wheelbase_;
    double dt_;
    int num_points_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathGeneratorNode>();

    // Create a separate thread for user input
    std::thread input_thread(&PathGeneratorNode::getInput, node);

    rclcpp::spin(node);

    // Join the input thread before shutting down
    if (input_thread.joinable()) {
        input_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
