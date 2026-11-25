#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "json_lite.hh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

class JointStateAdapter : public rclcpp::Node
{
public:
    JointStateAdapter() : Node("joint_state_adapter")
    {
        publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "/camera_fk/joint_states", 10);
        subscription_ = create_subscription<std_msgs::msg::String>(
            "/launcher/blue/state", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::String& msg) { publish_state(msg.data); });
    }

private:
    void publish_state(const std::string& json)
    {
        robomini::json::Value state;
        std::string error;
        if (!robomini::json::Parse(json, state, error) || !state.IsObject())
        {
            warn("invalid state JSON");
            return;
        }

        constexpr double missing = std::numeric_limits<double>::quiet_NaN();
        const double yaw = state.NumberOr("yaw", missing);
        const double pitch = state.NumberOr("pitch", missing);
        const double stamp = state.NumberOr("stamp", missing);
        constexpr std::int64_t ns_per_second = 1000000000;
        constexpr auto max_seconds = std::numeric_limits<std::int32_t>::max();
        if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(stamp) ||
            stamp < 0.0 || stamp >= static_cast<double>(max_seconds) + 1.0)
        {
            warn("expected finite yaw/pitch and a nonnegative, representable simulation stamp");
            return;
        }

        const auto nanoseconds = std::llround(stamp * ns_per_second);
        const auto seconds = nanoseconds / ns_per_second;
        if (seconds > max_seconds)
        {
            warn("simulation stamp overflows ROS time after rounding");
            return;
        }

        sensor_msgs::msg::JointState message;
        message.header.stamp.sec = static_cast<std::int32_t>(seconds);
        message.header.stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % ns_per_second);
        message.name = {"yaw_joint", "pitch_joint"};
        message.position = {yaw, pitch};
        publisher_->publish(message);
    }

    void warn(const char* reason)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), warning_clock_, 2000, "Skipping joint state: %s", reason);
    }

    rclcpp::Clock warning_clock_{RCL_STEADY_TIME};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateAdapter>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}
