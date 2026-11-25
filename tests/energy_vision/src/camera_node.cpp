
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("energy_camera_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &CameraNode::image_callback,
                this,
                std::placeholders::_1
            )
        );

        RCLCPP_INFO(this->get_logger(), "Camera node started!");
        RCLCPP_INFO(this->get_logger(), "Waiting for camera images...");
    }

private:
    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            // ROS Image -> OpenCV Mat
            auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

            // Display camera image
            cv::imshow("ROBOMINI Camera", cv_image->image);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image conversion failed: %s",
                e.what()
            );
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraNode>();

    rclcpp::spin(node);

    cv::destroyAllWindows();

    rclcpp::shutdown();

    return 0;
}
