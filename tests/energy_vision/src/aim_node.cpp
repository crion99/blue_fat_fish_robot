#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace energy_vision
{

class AimNode : public rclcpp::Node
{
public:
    AimNode() : Node("aim_node")
    {
        aim_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/aim/joint_target", 10);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/pending_target_pose", rclcpp::SensorDataQoS(),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_pose(*msg); });
            
        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/launcher/blue/state", 10,
            [this](std_msgs::msg::String::ConstSharedPtr msg) { on_state(*msg); });

        RCLCPP_INFO(get_logger(), "视觉中心伺服节点已启动 (Visual Servoing)");
    }

private:
    void on_state(const std_msgs::msg::String& msg)
    {
        auto parse_val = [](const std::string& s, const std::string& key) -> double {
            auto pos = s.find("\"" + key + "\":");
            if (pos == std::string::npos) return 0.0;
            pos += key.length() + 3;
            return std::stod(s.substr(pos));
        };
        current_yaw_ = parse_val(msg.data, "yaw");
        current_pitch_ = parse_val(msg.data, "pitch");
    }

    void on_pose(const geometry_msgs::msg::PoseStamped& msg)
    {
        // 靶点在相机光轴下的坐标
        const double cx = msg.pose.position.x; // 右正，左负
        const double cy = msg.pose.position.y; // 下正，上负
        const double cz = msg.pose.position.z; // 前正

        // 计算偏差
        // 【调整转向】如果云台追着靶子越跑越远（方向反了），就把这里的 -1.0 改成 1.0
        const double sign_yaw = -1.0; 
        const double sign_pitch = 1.0;

        double cam_yaw = sign_yaw * std::atan2(cx, cz);
        double cam_pitch = sign_pitch * std::atan2(-cy, std::hypot(cx, cz));

        // 目标角度 = 当前实际角度 + 相机偏差角
        double raw_target_yaw = current_yaw_ + cam_yaw;
        double raw_target_pitch = current_pitch_ + cam_pitch;

        // 低通滤波（平滑处理），减少卡顿和大幅度震荡
        if (!has_target_) {
            smoothed_target_yaw_ = raw_target_yaw;
            smoothed_target_pitch_ = raw_target_pitch;
            has_target_ = true;
        } else {
            smoothed_target_yaw_ = smoothed_target_yaw_ * 0.7 + raw_target_yaw * 0.3;
            smoothed_target_pitch_ = smoothed_target_pitch_ * 0.7 + raw_target_pitch * 0.3;
        }

        std_msgs::msg::Float64MultiArray out;
        out.data = {smoothed_target_yaw_, smoothed_target_pitch_, 1.0}; 
        aim_pub_->publish(out);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Cam偏差(%.2f, %.2f) -> 目标(%.2f, %.2f)",
            cam_yaw, cam_pitch, smoothed_target_yaw_, smoothed_target_pitch_);
    }

    double current_yaw_{0.0};
    double current_pitch_{0.0};
    bool has_target_{false};
    double smoothed_target_yaw_{0.0};
    double smoothed_target_pitch_{0.0};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr aim_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
};

}  // namespace energy_vision

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<energy_vision::AimNode>());
    rclcpp::shutdown();
    return 0;
}