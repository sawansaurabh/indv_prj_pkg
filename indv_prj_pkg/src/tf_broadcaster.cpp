#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdometryToTF : public rclcpp::Node
{
public:
  OdometryToTF()
  : Node("odometry_to_tf")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "unity_odom", 10, std::bind(&OdometryToTF::odom_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.pose.position.z;

    transform_stamped.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform_stamped);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToTF>());
  rclcpp::shutdown();
  return 0;
}
