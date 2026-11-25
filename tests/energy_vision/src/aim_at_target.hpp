#pragma once

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "two_axis_geometry.hpp"

namespace energy_vision
{

struct AimResult
{
    bool reachable;
    double yaw;                 // 弧度，绕 base_link +Z
    double pitch;               // 弧度，绕 yaw 后的 +X，可为负
    cv::Vec3d desired_dir;      // 期望指向，base_link 系单位向量
};

// 发射架参考点：pitch 关节轴心在 base_link 系位置。
// yaw_joint origin z=0.080 + pitch_joint origin z=0.290
// cv::Vec3d 不是字面类型，不能用 constexpr；C++17 的 inline const 保证唯一定义。
inline const cv::Vec3d kLauncherPivotInBase{0.0, 0.0, 0.370};

// 零位发射方向：base_link 系 +Z（与 yaw_pitch_rotation 约定一致）
inline const cv::Vec3d kZeroAimDirInBase{0.0, 0.0, 1.0};

constexpr JointRange kDefaultYawRange{-1.570796, 1.570796};
constexpr JointRange kDefaultPitchRange{-0.174533, 0.785398};
inline constexpr double kNormalRange = 1.0;  // 法向虚拟靶点距离，只影响方向

// 正向：由 (yaw,pitch) 得到单位指向。
//   d = Rz(yaw) * Rx(pitch) * (0,0,1)
//     = ( sin(pitch)*sin(yaw), -sin(pitch)*cos(yaw), cos(pitch) )
inline cv::Vec3d aim_direction_from_angles(double yaw, double pitch)
{
    const double sp = std::sin(pitch);
    return cv::Vec3d{sp * std::sin(yaw), -sp * std::cos(yaw), std::cos(pitch)};
}

// 给定 base_link 系三维靶点，求对准它所需的 yaw/pitch。
// 优先使用非负 pitch。两组等价解按关节限位筛选。
inline AimResult aim_at_target(
    const cv::Vec3d& target_in_base,
    JointRange yaw_range = kDefaultYawRange,
    JointRange pitch_range = kDefaultPitchRange)
{
    AimResult result{false, 0.0, 0.0, {0.0, 0.0, 0.0}};

    const cv::Vec3d delta = target_in_base - kLauncherPivotInBase;
    const double norm = cv::norm(delta);
    if (!(norm > 1e-9) || !std::isfinite(norm))
        return result;

    const cv::Vec3d d = delta / norm;
    result.desired_dir = d;

    const double horizontal = std::hypot(d[0], d[1]);

    const double pitch = std::atan2(horizontal, d[2]);
    const double yaw = (horizontal > 1e-9) ? std::atan2(d[0], -d[1]) : 0.0;
    result.yaw = yaw;
    result.pitch = pitch;
    result.reachable = yaw_range.contains(yaw) && pitch_range.contains(pitch);

    if (!result.reachable)
    {
        const double alternate_yaw =
            (horizontal > 1e-9) ? std::atan2(-d[0], d[1]) : 0.0;
        if (yaw_range.contains(alternate_yaw) && pitch_range.contains(-pitch))
        {
            result.yaw = alternate_yaw;
            result.pitch = -pitch;
            result.reachable = true;
        }
    }
    return result;
}

// 由相机光学系下的靶面法向 n_cam 和 base_link<-camera 的旋转 R_base_cam，
// 构造沿法向的 base_link 系虚拟靶点，供 aim_at_target 复用。
// 约定：法向朝相机一侧，即 n_cam.z >= 0；若为负则翻转。
inline cv::Vec3d normal_virtual_target(
    const cv::Vec3d& n_cam,
    const cv::Matx33d& R_base_cam,
    double range = kNormalRange)
{
    cv::Vec3d n = n_cam;
    const double norm = cv::norm(n);
    if (norm > 1e-9)
        n /= norm;
    if (n[2] < 0.0)
        n = -n;

    const cv::Vec3d n_base = R_base_cam * n;
    return kLauncherPivotInBase + n_base * range;
}

// Plate Tilt 诊断量，与 light_detector 的 arccos(|R(2,2)|) 一致，单位弧度。
inline double plate_tilt_rad(const cv::Matx33d& R_cam)
{
    return std::acos(std::clamp(std::abs(R_cam(2, 2)), 0.0, 1.0));
}

}  // namespace energy_vision
