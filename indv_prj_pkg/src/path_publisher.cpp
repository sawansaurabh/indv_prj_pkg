
#include <chrono>
#include <memory>
#include <functional>
#include <vector>
#include <cmath>
#include <utility> 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;

class PathPublisherNode : public rclcpp::Node {
public:
    PathPublisherNode() : Node("path_publisher") {
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", 10);

        // Create a timer to publish the path periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&PathPublisherNode::publishPath, this));
    }

private:
    vector<pair<double, double>> generateCircularPath(double radius, int numPoints) {
    vector<pair<double, double>> points;
    double thetaIncrement = 2 * M_PI / numPoints; // Angle increment between consecutive points

      for (int i = 0; i < numPoints; ++i) {
        double theta = i * thetaIncrement;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        points.push_back(make_pair(x, y));
      }
    return points;
    }

    void publishPath() {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = "map";  // Set the frame ID

        double radius = 2;
        int numPoints = 100;
        vector<pair<double, double>> path = generateCircularPath(radius, numPoints);

        for (auto& point : path) {
          geometry_msgs::msg::PoseStamped pose;

          pose.pose.position.x = point.first;
          pose.pose.position.y = point.second;

          path_msg.poses.push_back(pose);
        }

        // Publish the path
        publisher_->publish(path_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
