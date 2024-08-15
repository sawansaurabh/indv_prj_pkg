#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class DepthImageConverter : public rclcpp::Node {
public:
    DepthImageConverter()
        : Node("depth_image_converter") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image", 10,
            std::bind(&DepthImageConverter::image_callback, this, std::placeholders::_1));

        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat rgb_depth_image = cv_ptr->image;

        cv::Mat depth_image;
        cv::extractChannel(rgb_depth_image, depth_image, 0);
        
        // Normalize
        // cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);
        depth_image.convertTo(depth_image, CV_32FC1, 1.0/255.0); 

        sensor_msgs::msg::Image output_msg;
        output_msg.header = msg->header;
        output_msg.height = depth_image.rows;
        output_msg.width = depth_image.cols;
        output_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        output_msg.step = depth_image.cols * sizeof(float);
        output_msg.data.resize(depth_image.total() * depth_image.elemSize());
        memcpy(output_msg.data.data(), depth_image.data, output_msg.data.size());

        depth_image_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImageConverter>());
    rclcpp::shutdown();
    return 0;
}
