#include <cmath>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robomini_interfaces/msg/gimbal_to_vision.hpp"
#include "pose_feedback_checks.hpp"

class PoseFeedbackDiagnostic : public rclcpp::Node
{
public:
    PoseFeedbackDiagnostic()
        : Node("pose_feedback_diagnostic"),
          max_offset_ms_(declare_parameter<double>("max_offset_ms", 50.0)),
          feedback_timeout_ms_(declare_parameter<double>("feedback_timeout_ms", 250.0))
    {
        if (!std::isfinite(max_offset_ms_) || max_offset_ms_ < 0.0)
            throw std::invalid_argument("max_offset_ms must be finite and nonnegative");

        if (!std::isfinite(feedback_timeout_ms_) || feedback_timeout_ms_ <= 0.0)
            throw std::invalid_argument("feedback_timeout_ms must be finite and positive");

        feedback_sub_ = create_subscription<robomini_interfaces::msg::GimbalToVision>(
            "/gimbal_to_vision", rclcpp::SensorDataQoS(),
            [this](robomini_interfaces::msg::GimbalToVision::ConstSharedPtr message)
            {
                feedback_ = Feedback{now(), std::chrono::steady_clock::now(),
                    message->yaw, message->pitch};
            });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/pending_target_pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
            {
                report(*message);
            });
    }

private:
    struct Feedback
    {
        rclcpp::Time received;
        std::chrono::steady_clock::time_point steady_received;
        double yaw;
        double pitch;
    };

    void report(const geometry_msgs::msg::PoseStamped& message)
    {
        const auto current_time = now();
        const auto& p = message.pose.position;
        const auto& stamp = message.header.stamp;

        const char* status = "approximate";
        if (!get_clock()->ros_time_is_active() || current_time.nanoseconds() == 0)
            status = "waiting_for_sim_clock";
        else if (message.header.frame_id.empty() || stamp.sec < 0 ||
            stamp.nanosec >= 1000000000u || (stamp.sec == 0 && stamp.nanosec == 0) ||
            !energy_vision::is_visible_camera_point(p.x, p.y, p.z))
            status = "invalid_pose";
        else if (!feedback_ || feedback_->received.nanoseconds() == 0)
            status = "waiting_for_feedback";
        else if (feedback_->received > current_time)
        {
            feedback_.reset();
            status = "clock_reset";
        }
        else
        {
            const rclcpp::Time image_time(stamp, current_time.get_clock_type());
            const double offset_ms = (feedback_->received - image_time).seconds() * 1000.0;
            const double age_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - feedback_->steady_received).count();
            status = energy_vision::feedback_status(
                feedback_->yaw, feedback_->pitch, age_ms, feedback_timeout_ms_,
                offset_ms, max_offset_ms_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "status=%s frame=%s camera_xyz_m=(%.3f, %.3f, %.3f) "
                "feedback_yaw_rad=%.4f feedback_pitch_rad=%.4f "
                "feedback_yaw_deg=%.2f feedback_pitch_deg=%.2f "
                "feedback_age_ms=%.1f receipt_minus_image_ms=%+.1f",
                status, message.header.frame_id.c_str(), p.x, p.y, p.z,
                feedback_->yaw, feedback_->pitch,
                energy_vision::radians_to_degrees(feedback_->yaw),
                energy_vision::radians_to_degrees(feedback_->pitch),
                age_ms, offset_ms);
            return;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "status=%s frame=%s", status, message.header.frame_id.c_str());
    }

    const double max_offset_ms_;
    const double feedback_timeout_ms_;
    std::optional<Feedback> feedback_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<robomini_interfaces::msg::GimbalToVision>::SharedPtr feedback_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<PoseFeedbackDiagnostic>());
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "%s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
