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
    declare_parameter("cam_fwd_m", 0.036);   // 相机光心相对俯仰轴：沿枪管向前（m）
    declare_parameter("cam_up_m", 0.075);    // 相机光心相对俯仰轴：向上（m）
    declare_parameter("muzzle_fwd_m", 0.095);// 枪口相对俯仰轴：沿枪管向前（m）
    declare_parameter("fire_mode", 1);      // 0=只跟不射 1=识别到就射 2=对准才射
    declare_parameter("fire_align_deg", 1.5);  // 锁定开火：云台追平指令角（含提前量）的阈值（度）
    declare_parameter("gravity", 9.81);     // 重力补偿（m/s²），0 表示关闭
    declare_parameter("pitch_offset_deg", 0.0);  // 实测标定的固定仰角偏置（度）
    declare_parameter("latency_s", 0.03);   // 图像到出弹总延迟（s）
    declare_parameter("stale_timeout_s", 0.40);  // 超过该时长无目标则发 mode=0
    declare_parameter("dead_reckon_s", 0.50);    // 丢失后按角速度航位推算的持续时长
    declare_parameter("smooth_alpha", 0.3);      // 指令低通系数，越小越“钝”越不易过冲
    declare_parameter("max_cmd_rate_radps", 1.5);  // 单帧限速，防止猛摆冲出视野
    declare_parameter("input_alpha", 0.35);      // 输入位置低通（抑制检测框抖动）
    declare_parameter("lead_gain", 0.0);         // 提前量增益：0=关闭（默认，避免转过头）
    declare_parameter("ab_alpha", 0.45);         // alpha-beta 滤波：位置增益
    declare_parameter("ab_beta", 0.08);          // alpha-beta 滤波：速度增益
    declare_parameter("jump_reset_deg", 15.0);   // 目标跳变超过该角度就复位（换靶）
    declare_parameter("lock_deg", 0.5);          // 误差小于该值并持续 lock_frames 帧 -> 锁定冻结
    declare_parameter("unlock_deg", 2.0);        // 误差超过该值 -> 解锁（迟滞，防边界抖）
    declare_parameter("lock_frames", 5);         // 连续多少帧满足才锁定
    declare_parameter("deadband_deg", 0.2);      // 死区：指令变化小于该值就不动
    declare_parameter("yaw_sign", 1.0);          // 云台 yaw 方向：转向反了就设 -1
    declare_parameter("pitch_sign", 1.0);        // 云台 pitch 方向：反了就设 -1

    cam_fwd_ = get_parameter("cam_fwd_m").as_double();
    cam_up_ = get_parameter("cam_up_m").as_double();
    muzzle_fwd_ = get_parameter("muzzle_fwd_m").as_double();
    fire_mode_ = get_parameter("fire_mode").as_int();
    fire_align_deg_ = get_parameter("fire_align_deg").as_double();
    gravity_ = get_parameter("gravity").as_double();
    pitch_offset_deg_ = get_parameter("pitch_offset_deg").as_double();
    latency_s_ = get_parameter("latency_s").as_double();
    stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();
    dead_reckon_s_ = get_parameter("dead_reckon_s").as_double();
    smooth_alpha_ = get_parameter("smooth_alpha").as_double();
    max_cmd_rate_radps_ = get_parameter("max_cmd_rate_radps").as_double();
    input_alpha_ = get_parameter("input_alpha").as_double();
    deadband_rad_ = get_parameter("deadband_deg").as_double() * kDeg2Rad;
    lead_gain_ = get_parameter("lead_gain").as_double();
    ab_alpha_ = get_parameter("ab_alpha").as_double();
    ab_beta_ = get_parameter("ab_beta").as_double();
    jump_reset_rad_ = get_parameter("jump_reset_deg").as_double() * kDeg2Rad;
    lock_rad_ = get_parameter("lock_deg").as_double() * kDeg2Rad;
    unlock_rad_ = get_parameter("unlock_deg").as_double() * kDeg2Rad;
    lock_frames_ = get_parameter("lock_frames").as_int();
    yaw_sign_ = get_parameter("yaw_sign").as_double();
    pitch_sign_ = get_parameter("pitch_sign").as_double();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vision/pending_target_pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onPose(*msg); });

    gimbal_sub_ = create_subscription<robomini_interfaces::msg::GimbalToVision>(
      "/gimbal_to_vision", rclcpp::SensorDataQoS(),
      [this](robomini_interfaces::msg::GimbalToVision::ConstSharedPtr msg) { onGimbal(*msg); });

    cmd_pub_ = create_publisher<robomini_interfaces::msg::VisionToGimbal>("/vision_to_gimbal", 10);

    // 低速率兜底：目标丢失超时后发 mode=0，让云台保持/停止而不是悬着旧指令
    stale_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { onStaleCheck(); });

    RCLCPP_INFO(get_logger(), "pose_aim_node 启动：cam(%.3f,%.3f) muzzle=%.3f fire_mode=%d",
                cam_fwd_, cam_up_, muzzle_fwd_, fire_mode_);
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

    // 输入低通：检测框中心/距离逐帧抖动，直接算角会让云台跟着抖
    if (!has_input_) {
      fx_ = cx;
      fy_ = cy;
      fz_ = cz;
      has_input_ = true;
    } else {
      fx_ += input_alpha_ * (cx - fx_);
      fy_ += input_alpha_ * (cy - fy_);
      fz_ += input_alpha_ * (cz - fz_);
    }

    // 用“收到消息的时刻”记时间戳，而不是消息里的 header.stamp。
    // 关键：msg.header.stamp 是仿真时间，而 get_clock()->now() 在没开 use_sim_time 时是墙钟时间，
    // 两者相减会差 ~1.7e9 秒，导致“目标丢失”误报。统一用 get_clock()->now() 后，
    // 无论是否开 use_sim_time 都在同一个时间域，判停逻辑才是对的。
    last_pose_time_ = get_clock()->now();

    // 相机系 -> 云台系（云台系：x 沿枪管向前、y 向左、z 向上）。
    // 相机光心在 pitch_link 内位于 (cam_fwd, 0, cam_up)，枪口在 (muzzle_fwd, 0, 0)。
    // 靶心相机系坐标 (cx 右, cy 下, cz 前) 转云台系方向为 (cz 前, -cx 左, -cy 上)，
    // 因此 枪口 -> 靶心 = (cam_fwd + cz - muzzle_fwd, -cx, cam_up - cy)。
    const double dx = cam_fwd_ + fz_ - muzzle_fwd_;
    const double dy = -fx_;
    const double dz = cam_up_ - fy_;

    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    // 方向符号可配：若云台“追着靶子越跑越远”，把对应 *_sign 设成 -1
    const double rel_yaw = std::atan2(dy, dx) * yaw_sign_;
    const double rel_pitch = std::atan2(dz, std::hypot(dx, dy)) * pitch_sign_;

    double raw_yaw = yaw_ + rel_yaw;
    double raw_pitch = pitch_ + rel_pitch;

    // 重力补偿：球在飞行途中下坠，抬高仰角。dx/dy 是云台系里的水平方向分量
    if (gravity_ > 0.0) {
      const double d_horiz = std::hypot(dx, dy);
      const double t_flight = d_horiz / bullet_speed_;
      const double drop = 0.5 * gravity_ * t_flight * t_flight;
      raw_pitch += std::atan2(drop, d_horiz);
    }

    // 实测标定的固定仰角偏置（球速/安装误差的一揽子补偿，默认 0）
    raw_pitch += pitch_offset_deg_ * M_PI / 180.0;

    // 旋转靶提前量（默认关闭：yaw_rate_ 是“云台自己”的角速度，
    // 直接拿它做提前量会变成正反馈，云台越快越往前冲，表现就是“转过头”反复摆）
    const double flight = distance / bullet_speed_;
    if (lead_gain_ > 0.0) {
      raw_yaw += lead_gain_ * yaw_rate_ * (latency_s_ + flight);
    }

    // alpha-beta 滤波（比 EMA 多一个速度状态：更顺、滞后更小；换靶时自动复位）
    const rclcpp::Time now_pose = get_clock()->now();
    if (!ab_init_) {
      ab_yaw_ = raw_yaw;
      ab_pitch_ = raw_pitch;
      ab_yaw_rate_ = 0.0;
      ab_pitch_rate_ = 0.0;
      ab_stamp_ = now_pose;
      ab_init_ = true;
    } else {
      double dt = (now_pose - ab_stamp_).seconds();
      ab_stamp_ = now_pose;
      if (dt <= 1e-4 || dt > 0.5) dt = 1.0 / 60.0;
      const double pred_y = wrapAngle(ab_yaw_ + ab_yaw_rate_ * dt);
      const double pred_p = ab_pitch_ + ab_pitch_rate_ * dt;
      const double res_y = wrapAngle(raw_yaw - pred_y);
      const double res_p = raw_pitch - pred_p;
      if (std::fabs(res_y) > jump_reset_rad_ || std::fabs(res_p) > jump_reset_rad_) {
        // 换靶（5 个靶相隔 72°）：直接复位，不让滤波器拖着走
        ab_yaw_ = raw_yaw;
        ab_pitch_ = raw_pitch;
        ab_yaw_rate_ = 0.0;
        ab_pitch_rate_ = 0.0;
      } else {
        ab_yaw_ = wrapAngle(pred_y + ab_alpha_ * res_y);
        ab_pitch_ = pred_p + ab_alpha_ * res_p;
        ab_yaw_rate_ += ab_beta_ * res_y / dt;
        ab_pitch_rate_ += ab_beta_ * res_p / dt;
      }
    }
    smoothed_yaw_ = ab_yaw_;
    smoothed_pitch_ = ab_pitch_;

    // 限速：单帧允许的最大角度变化，防止一帧猛摆几十度把目标甩出视野
    const double dt = 1.0 / 60.0;
    const double max_step = max_cmd_rate_radps_ * dt;
    double d_yaw = wrapAngle(smoothed_yaw_ - last_cmd_yaw_);
    double d_pitch = smoothed_pitch_ - last_cmd_pitch_;
    d_yaw = std::clamp(d_yaw, -max_step, max_step);
    d_pitch = std::clamp(d_pitch, -max_step, max_step);
    // 静止靶锁定：连续几帧误差都很小就冻结指令，让云台真正停住（不再逐帧微调）；
    // 误差超过 unlock_deg 才解锁（迟滞，避免在阈值边界反复进出）
    if (locked_) {
      if (std::fabs(rel_yaw) > unlock_rad_ || std::fabs(rel_pitch) > unlock_rad_) {
        locked_ = false;
        lock_count_ = 0;
      }
    } else if (std::fabs(rel_yaw) < lock_rad_ && std::fabs(rel_pitch) < lock_rad_) {
      if (++lock_count_ >= lock_frames_) locked_ = true;
    } else {
      lock_count_ = 0;
    }

    const double yaw_cmd0 = last_cmd_yaw_ + d_yaw;
    const double pitch_cmd0 = last_cmd_pitch_ + d_pitch;
    // 死区 + 锁定：指令不变化就保持上一次输出，云台静止
    double yaw_cmd = yaw_cmd0;
    double pitch_cmd = pitch_cmd0;
    const bool frozen = locked_ ||
        (std::fabs(wrapAngle(yaw_cmd0 - last_cmd_yaw_)) < deadband_rad_ &&
         std::fabs(pitch_cmd0 - last_cmd_pitch_) < deadband_rad_);
    if (!frozen) {
      last_cmd_yaw_ = yaw_cmd0;
      last_cmd_pitch_ = pitch_cmd0;
    } else {
      yaw_cmd = last_cmd_yaw_;
      pitch_cmd = last_cmd_pitch_;
    }

    auto cmd = robomini_interfaces::msg::VisionToGimbal();
    // fire_mode 语义（对外）：0=只跟不射；1=识别到就射；2=对准才射
    // 注意 VisionToGimbal.mode 的语义是 0=不控制 / 1=控制不射 / 2=控制并开火，
    // 两者不是一回事，必须显式换算（之前的写法把 1 直接当 mode，导致“跟踪即开火”其实不开火）。
    uint8_t fire_cmd = 1;  // 默认：控制云台但不发射
    if (fire_mode_ == 1) {
      fire_cmd = 2;        // 识别到目标就开火
    } else if (fire_mode_ == 2) {
      const double align = fire_align_deg_ * M_PI / 180.0;
      const bool caught_up = std::fabs(wrapAngle(yaw_ - last_cmd_yaw_)) < align &&
                             std::fabs(pitch_ - last_cmd_pitch_) < align;
      const bool centered = std::fabs(rel_yaw) < align && std::fabs(rel_pitch) < align;
      fire_cmd = (caught_up || centered) ? 2 : 1;
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
    ab_init_ = false;          // 滤波也复位，避免用旧的预测值
    locked_ = false;           // 解锁，重新捕获后重新判锁定
    lock_count_ = 0;
    last_cmd_yaw_ = yaw_;
    last_cmd_pitch_ = pitch_;
    cmd_pub_->publish(cmd);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "目标丢失 %.1f s，发送 mode=0", age);
  }

  double cam_fwd_{0.036};
  double cam_up_{0.075};
  double muzzle_fwd_{0.095};
  int fire_mode_{1};
  double fire_align_deg_{1.5};
  double gravity_{9.81};
  double pitch_offset_deg_{0.0};
  double latency_s_{0.03};
  double stale_timeout_s_{0.40};
  double dead_reckon_s_{0.50};
  double smooth_alpha_{0.3};
  double max_cmd_rate_radps_{1.5};
  double input_alpha_{0.35};
  double deadband_rad_{0.2 * kDeg2Rad};
  double lead_gain_{0.0};
  double ab_alpha_{0.45};
  double ab_beta_{0.08};
  double jump_reset_rad_{15.0 * kDeg2Rad};
  bool ab_init_{false};
  double ab_yaw_{0.0};
  double ab_pitch_{0.0};
  double ab_yaw_rate_{0.0};
  double ab_pitch_rate_{0.0};
  rclcpp::Time ab_stamp_;
  double lock_rad_{0.5 * kDeg2Rad};
  double unlock_rad_{2.0 * kDeg2Rad};
  int lock_frames_{5};
  bool locked_{false};
  int lock_count_{0};
  double yaw_sign_{1.0};
  double pitch_sign_{1.0};
  bool has_input_{false};
  double fx_{0.0};
  double fy_{0.0};
  double fz_{0.0};
  bool has_cmd_{false};
  double smoothed_yaw_{0.0};
  double smoothed_pitch_{0.0};
  double last_cmd_yaw_{0.0};
  double last_cmd_pitch_{0.0};

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
