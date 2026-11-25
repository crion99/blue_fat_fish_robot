#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "json_lite.hh"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using Array = std_msgs::msg::Float64MultiArray;
using String = std_msgs::msg::String;

constexpr double kReachedTolerance = 0.5 * 3.14159265358979323846 / 180.0;
constexpr double kMissing = std::numeric_limits<double>::quiet_NaN();

struct JointAngles
{
    double yaw;
    double pitch;
};

class AimFollower : public rclcpp::Node
{
public:
    AimFollower() : Node("aim_follower")
    {
        gimbal_pub_ = create_publisher<String>("/launcher/blue/gimbal", 10);
        status_pub_ = create_publisher<Array>("/aim/status", 10);
        target_sub_ = create_subscription<Array>(
            "/aim/joint_target", 10,
            [this](const Array& msg) { on_target(msg); });
        state_sub_ = create_subscription<String>(
            "/launcher/blue/state", rclcpp::SensorDataQoS(),
            [this](const String& msg) { on_state(msg.data); });
    }

private:
    void on_target(const Array& msg)
    {
        const auto& data = msg.data;
        if (data.size() != 3 || data[2] != 1.0 ||
            !std::isfinite(data[0]) || !std::isfinite(data[1]))
        {
            target_.reset();
            publish_status(false, kMissing, kMissing);
            return;
        }

        target_ = JointAngles{data[0], data[1]};
        std::ostringstream json;
        json.imbue(std::locale::classic());
        json << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "{\"yaw\":" << target_->yaw << ",\"pitch\":" << target_->pitch << '}';
        String command;
        command.data = json.str();
        gimbal_pub_->publish(command);
    }

    void on_state(const std::string& json)
    {
        if (!target_)
            return;

        robomini::json::Value state;
        std::string error;
        if (!robomini::json::Parse(json, state, error) || !state.IsObject())
            return;

        const double yaw = state.NumberOr("yaw", kMissing);
        const double pitch = state.NumberOr("pitch", kMissing);
        if (!std::isfinite(yaw) || !std::isfinite(pitch))
            return;

        const double yaw_error = std::abs(yaw - target_->yaw);
        const double pitch_error = std::abs(pitch - target_->pitch);
        publish_status(
            yaw_error < kReachedTolerance && pitch_error < kReachedTolerance,
            yaw_error, pitch_error);
    }

    void publish_status(bool reached, double yaw_error, double pitch_error)
    {
        Array status;
        status.data = {reached ? 1.0 : 0.0, yaw_error, pitch_error};
        status_pub_->publish(status);
    }

    std::optional<JointAngles> target_;
    rclcpp::Publisher<String>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<Array>::SharedPtr status_pub_;
    rclcpp::Subscription<Array>::SharedPtr target_sub_;
    rclcpp::Subscription<String>::SharedPtr state_sub_;
};
}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AimFollower>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}
