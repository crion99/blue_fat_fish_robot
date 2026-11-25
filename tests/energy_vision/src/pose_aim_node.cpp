// 第三部分（瞄准）：把 light_detector 解出的靶心 3D 位置换算成云台角度指令。
//
// 输入：
//   /vision/pending_target_pose   geometry_msgs/PoseStamped（靶心在相机光轴系：
//                                   position.x=右、y=下、z=前，米；与 light_detector 一致）
//   /gimbal_to_vision             robomini_interfaces/GimbalToVision（当前 yaw/pitch/角速度/弹速）
// 输出：
//   /vision_to_gimbal             robomini_interfaces/VisionToGimbal
//
// 坐标链（详见 algorithm/docs/03-靶位姿解算与瞄准转角.md）：
//   相机系 -> 云台系是固定旋转（相机与枪管同向）：
//       相机 x(右) -> 云台 -y；相机 y(下) -> 云台 -z；相机 z(前) -> 云台 +x
//   靶心在云台系 p_g = (z + t_cam, -x, -y)，枪口在 (t_muzzle, 0, 0)，
//   瞄准方向 d = p_g - (t_muzzle, 0, 0)，
//       yaw   = atan2(d_y, d_x)
//       pitch = atan2(d_z, hypot(d_x, d_y))
//   目标绝对角 = 当前云台角 + 上述相对角，再叠加旋转靶提前量 yaw_rate·(latency + d/v)。

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robomini_interfaces/msg/gimbal_to_vision.hpp"
#include "robomini_interfaces/msg/vision_to_gimbal.hpp"

namespace energy_vision
{

class PoseAimNode : public rclcpp::Node
{
public:
  PoseAimNode() : Node("pose_aim_node")
  {
    declare_parameter("t_cam_x", 0.00);     // 相机相对云台旋转中心的沿枪管偏移（m）
    declare_parameter("t_muzzle_x", 0.10);  // 枪口沿枪管方向的偏移（m）
    declare_parameter("fire_mode", 1);      // 1=只跟瞄 2=跟瞄并开火
    declare_parameter("latency_s", 0.03);   // 图像到出弹总延迟（s）
    declare_parameter("stale_timeout_s", 0.20);  // 超过该时长无目标则发 mode=0

    t_cam_x_ = get_parameter("t_cam_x").as_double();
    t_muzzle_x_ = get_parameter("t_muzzle_x").as_double();
    fire_mode_ = get_parameter("fire_mode").as_int();
    latency_s_ = get_parameter("latency_s").as_double();
    stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/pending_target_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onPose(*msg); });

    gimbal_sub_ = create_subscription<robomini_interfaces::msg::GimbalToVision>(
      "/gimbal_to_vision", rclcpp::SensorDataQoS(),
      [this](robomini_interfaces::msg::GimbalToVision::ConstSharedPtr msg) { onGimbal(*msg); });

    cmd_pub_ = create_publisher<robomini_interfaces::msg::VisionToGimbal>("/vision_to_gimbal", 10);

    // 低速率兜底：目标丢失超时后发 mode=0，让云台保持/停止而不是悬着旧指令
    stale_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { onStaleCheck(); });

    RCLCPP_INFO(get_logger(), "pose_aim_node 启动：t_cam=%.3f t_muzzle=%.3f fire_mode=%d",
                t_cam_x_, t_muzzle_x_, fire_mode_);
  }

private:
  void onGimbal(const robomini_interfaces::msg::GimbalToVision & msg)
  {
    yaw_ = msg.yaw;
    pitch_ = msg.pitch;
    yaw_rate_ = msg.yaw_vel;
    bullet_speed_ = msg.bullet_speed > 1.0 ? msg.bullet_speed : 12.0;
  }

  void onPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    const double cx = msg.pose.position.x;  // 右正
    const double cy = msg.pose.position.y;  // 下正
    const double cz = msg.pose.position.z;  // 前正

    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz) || cz <= 0.05) return;

    last_pose_time_ = rclcpp::Time(msg.header.stamp);

    // 相机系 -> 云台系
    const double gx = cz + t_cam_x_;
    const double gy = -cx;
    const double gz = -cy;

    // 枪口偏移：瞄准方向从枪口出发
    const double dx = gx - t_muzzle_x_;
    const double dy = gy;
    const double dz = gz;

    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double rel_yaw = std::atan2(dy, dx);
    const double rel_pitch = std::atan2(dz, std::hypot(dx, dy));

    double yaw_cmd = yaw_ + rel_yaw;
    double pitch_cmd = pitch_ + rel_pitch;

    // 旋转靶提前量
    const double flight = distance / bullet_speed_;
    yaw_cmd += yaw_rate_ * (latency_s_ + flight);

    auto cmd = robomini_interfaces::msg::VisionToGimbal();
    cmd.mode = static_cast<uint8_t>(fire_mode_);
    cmd.yaw = yaw_cmd;
    cmd.pitch = pitch_cmd;
    cmd.yaw_vel = yaw_rate_;
    cmd.yaw_acc = 0.0;
    cmd.pitch_vel = 0.0;
    cmd.pitch_acc = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "d=%.2fm rel(%.1f°, %.1f°) -> yaw %.1f° pitch %.1f°", distance,
                         rel_yaw * 180.0 / M_PI, rel_pitch * 180.0 / M_PI,
                         yaw_cmd * 180.0 / M_PI, pitch_cmd * 180.0 / M_PI);
  }

  void onStaleCheck()
  {
    if (last_pose_time_.nanoseconds() == 0) return;  // 从未收到目标，不干预
    const rclcpp::Time now = get_clock()->now();
    if ((now - last_pose_time_).seconds() <= stale_timeout_s_) return;

    auto cmd = robomini_interfaces::msg::VisionToGimbal();
    cmd.mode = 0;  // 不控制，云台保持
    cmd.yaw = yaw_;
    cmd.pitch = pitch_;
    cmd.yaw_vel = 0.0;
    cmd.yaw_acc = 0.0;
    cmd.pitch_vel = 0.0;
    cmd.pitch_acc = 0.0;
    cmd_pub_->publish(cmd);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "目标丢失 %.1f s，发送 mode=0",
                         (now - last_pose_time_).seconds());
  }

  double t_cam_x_{0.0};
  double t_muzzle_x_{0.10};
  int fire_mode_{1};
  double latency_s_{0.03};
  double stale_timeout_s_{0.20};

  double yaw_{0.0};
  double pitch_{0.0};
  double yaw_rate_{0.0};
  double bullet_speed_{12.0};

  rclcpp::Time last_pose_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<robomini_interfaces::msg::GimbalToVision>::SharedPtr gimbal_sub_;
  rclcpp::Publisher<robomini_interfaces::msg::VisionToGimbal>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
};

}  // namespace energy_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<energy_vision::PoseAimNode>());
  rclcpp::shutdown();
  return 0;
}
