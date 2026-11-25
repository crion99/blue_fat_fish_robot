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

#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robomini_interfaces/msg/gimbal_to_vision.hpp"
#include "robomini_interfaces/msg/vision_to_gimbal.hpp"

namespace energy_vision
{
namespace {
double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}
}  // namespace

class PoseAimNode : public rclcpp::Node
{
public:
  PoseAimNode() : Node("pose_aim_node")
  {
    declare_parameter("t_cam_x", 0.00);     // 相机相对云台旋转中心的沿枪管偏移（m）
    declare_parameter("t_muzzle_x", 0.10);  // 枪口沿枪管方向的偏移（m）
    declare_parameter("fire_mode", 2);      // 0=只跟 1=跟踪即开火 2=锁定后开火（默认）
    declare_parameter("fire_align_deg", 1.0);     // 锁定开火：靶心与光轴夹角低于此值（度）
    declare_parameter("fire_settle_degps", 2.0);  // 锁定开火：云台角速度低于此值（度/秒）
    declare_parameter("latency_s", 0.03);   // 图像到出弹总延迟（s）
    declare_parameter("stale_timeout_s", 0.40);  // 超过该时长无目标则发 mode=0
    declare_parameter("dead_reckon_s", 0.50);    // 丢失后按角速度航位推算的持续时长
    declare_parameter("smooth_alpha", 0.4);      // 指令低通系数，越小越“钝”越不易过冲
    declare_parameter("max_cmd_rate_radps", 3.0);  // 单帧限速，防止猛摆冲出视野

    t_cam_x_ = get_parameter("t_cam_x").as_double();
    t_muzzle_x_ = get_parameter("t_muzzle_x").as_double();
    fire_mode_ = get_parameter("fire_mode").as_int();
    fire_align_deg_ = get_parameter("fire_align_deg").as_double();
    fire_settle_degps_ = get_parameter("fire_settle_degps").as_double();
    latency_s_ = get_parameter("latency_s").as_double();
    stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();
    dead_reckon_s_ = get_parameter("dead_reckon_s").as_double();
    smooth_alpha_ = get_parameter("smooth_alpha").as_double();
    max_cmd_rate_radps_ = get_parameter("max_cmd_rate_radps").as_double();

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
    pitch_rate_ = msg.pitch_vel;
    bullet_speed_ = msg.bullet_speed > 1.0 ? msg.bullet_speed : 12.0;
  }

  void onPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    const double cx = msg.pose.position.x;  // 右正
    const double cy = msg.pose.position.y;  // 下正
    const double cz = msg.pose.position.z;  // 前正

    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz) || cz <= 0.05) return;

    // 用“收到消息的时刻”记时间戳，而不是消息里的 header.stamp。
    // 关键：msg.header.stamp 是仿真时间，而 get_clock()->now() 在没开 use_sim_time 时是墙钟时间，
    // 两者相减会差 ~1.7e9 秒，导致“目标丢失”误报。统一用 get_clock()->now() 后，
    // 无论是否开 use_sim_time 都在同一个时间域，判停逻辑才是对的。
    last_pose_time_ = get_clock()->now();

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

    double raw_yaw = yaw_ + rel_yaw;
    double raw_pitch = pitch_ + rel_pitch;

    // 旋转靶提前量（先加在原始目标上，再一起平滑）
    const double flight = distance / bullet_speed_;
    raw_yaw += yaw_rate_ * (latency_s_ + flight);

    // 低通平滑：抑制像素噪声 + 反馈延迟引起的抖振/过冲
    if (!has_cmd_) {
      smoothed_yaw_ = raw_yaw;
      smoothed_pitch_ = raw_pitch;
      has_cmd_ = true;
    } else {
      smoothed_yaw_ += smooth_alpha_ * wrapAngle(raw_yaw - smoothed_yaw_);
      smoothed_pitch_ += smooth_alpha_ * (raw_pitch - smoothed_pitch_);
    }

    // 限速：单帧允许的最大角度变化，防止一帧猛摆几十度把目标甩出视野
    const double dt = 1.0 / 60.0;
    const double max_step = max_cmd_rate_radps_ * dt;
    double d_yaw = wrapAngle(smoothed_yaw_ - last_cmd_yaw_);
    double d_pitch = smoothed_pitch_ - last_cmd_pitch_;
    d_yaw = std::clamp(d_yaw, -max_step, max_step);
    d_pitch = std::clamp(d_pitch, -max_step, max_step);
    const double yaw_cmd = last_cmd_yaw_ + d_yaw;
    const double pitch_cmd = last_cmd_pitch_ + d_pitch;
    last_cmd_yaw_ = yaw_cmd;
    last_cmd_pitch_ = pitch_cmd;

    auto cmd = robomini_interfaces::msg::VisionToGimbal();
    uint8_t fire_cmd = static_cast<uint8_t>(std::clamp(fire_mode_, 0, 2));
    if (fire_cmd == 2) {
      // 锁定才开火：对准 + 云台基本停稳，避免边扫边射浪费弹药/吃超发罚分
      const double align = fire_align_deg_ * M_PI / 180.0;
      const double settle = fire_settle_degps_ * M_PI / 180.0;
      const bool locked = std::fabs(rel_yaw) < align && std::fabs(rel_pitch) < align &&
                          std::fabs(yaw_rate_) < settle && std::fabs(pitch_rate_) < settle;
      if (!locked) fire_cmd = 1;
    }
    cmd.mode = fire_cmd;
    cmd.yaw = yaw_cmd;
    cmd.pitch = pitch_cmd;
    cmd.yaw_vel = 0.0;   // 只做位置跟踪，速度前馈交给仿真/下位机自己的控制器，避免二次叠加
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
    const double age = (now - last_pose_time_).seconds();
    if (age <= stale_timeout_s_) return;

    // 短时丢失：按最近角速度航位推算，继续保持扫动，等靶子转回视野，而不是立刻停住
    if (age <= stale_timeout_s_ + dead_reckon_s_ && std::fabs(yaw_rate_) > 0.05) {
      const double elapsed = age - stale_timeout_s_;
      auto cmd = robomini_interfaces::msg::VisionToGimbal();
      cmd.mode = 1;  // 只跟不射
      cmd.yaw = last_cmd_yaw_ + yaw_rate_ * elapsed;
      cmd.pitch = last_cmd_pitch_;
      cmd.yaw_vel = 0.0;
      cmd.yaw_acc = 0.0;
      cmd.pitch_vel = 0.0;
      cmd.pitch_acc = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    auto cmd = robomini_interfaces::msg::VisionToGimbal();
    cmd.mode = 0;  // 不控制，云台保持
    cmd.yaw = yaw_;
    cmd.pitch = pitch_;
    cmd.yaw_vel = 0.0;
    cmd.yaw_acc = 0.0;
    cmd.pitch_vel = 0.0;
    cmd.pitch_acc = 0.0;
    has_cmd_ = false;          // 目标丢失后重置平滑/限速状态，重新捕获时从当前姿态起步
    last_cmd_yaw_ = yaw_;
    last_cmd_pitch_ = pitch_;
    cmd_pub_->publish(cmd);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "目标丢失 %.1f s，发送 mode=0", age);
  }

  double t_cam_x_{0.0};
  double t_muzzle_x_{0.10};
  int fire_mode_{2};
  double fire_align_deg_{1.0};
  double fire_settle_degps_{2.0};
  double latency_s_{0.03};
  double stale_timeout_s_{0.40};
  double dead_reckon_s_{0.50};
  double smooth_alpha_{0.4};
  double max_cmd_rate_radps_{3.0};
  bool has_cmd_{false};
  double smoothed_yaw_{0.0};
  double smoothed_pitch_{0.0};
  double last_cmd_yaw_{0.0};
  double last_cmd_pitch_{0.0};

  double yaw_{0.0};
  double pitch_{0.0};
  double yaw_rate_{0.0};
  double pitch_rate_{0.0};
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
