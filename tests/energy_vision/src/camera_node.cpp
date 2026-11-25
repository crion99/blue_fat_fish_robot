// 第二部分：相机采集 + 实时显示（含帧率、截图、录制）。
//
// 运行：ros2 run energy_vision camera_node
// 按键：q/ESC 退出，s 截图 snapshot.png，r 开始/结束录制 camera.avi

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("energy_camera_node")
    {
        record_path_ = declare_parameter<std::string>("record_path", "camera.avi");
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Camera node started!  (q=quit, s=snapshot, r=record %s)",
                    record_path_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_image->image;

            // 帧率：最近 60 帧的滑动窗口
            const auto now = std::chrono::steady_clock::now();
            double fps = 0.0;
            if (times_.size() >= 2) {
                const double dt = std::chrono::duration<double>(now - times_.front()).count();
                if (dt > 1e-6) fps = static_cast<double>(times_.size() - 1) / dt;
            }
            times_.push_back(now);
            while (times_.size() > 60) times_.pop_front();

            cv::putText(frame, cv::format("%.1f fps  %dx%d", fps, frame.cols, frame.rows),
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        cv::Scalar(0, 255, 0), 2);

            cv::imshow("ROBOMINI Camera", frame);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) {
                rclcpp::shutdown();
            } else if (key == 's') {
                cv::imwrite("snapshot.png", frame);
                RCLCPP_INFO(this->get_logger(), "截图已保存 snapshot.png");
            } else if (key == 'r') {
                if (writer_.isOpened()) {
                    writer_.release();
                    RCLCPP_INFO(this->get_logger(), "录制结束");
                } else {
                    writer_.open(record_path_,
                                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0,
                                 cv::Size(frame.cols, frame.rows));
                    RCLCPP_INFO(this->get_logger(), "开始录制 -> %s", record_path_.c_str());
                }
            }
            if (writer_.isOpened()) writer_.write(frame);
        }
        catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter writer_;
    std::deque<std::chrono::steady_clock::time_point> times_;
    std::string record_path_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    node.reset();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
