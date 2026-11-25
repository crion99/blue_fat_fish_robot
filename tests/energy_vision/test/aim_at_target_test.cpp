#include <algorithm>
#include <cmath>
#include <cstdio>
#include <opencv2/core.hpp>

#include "aim_at_target.hpp"

using namespace energy_vision;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kAimTolDeg = 0.1;

int g_failures = 0;

void expect_near(const char* name, double actual, double expected, double tol)
{
    const double err = std::abs(actual - expected);
    if (err > tol)
    {
        std::printf("[FAIL] %s: actual=%.9f expected=%.9f err=%.9f\n",
            name, actual, expected, err);
        ++g_failures;
    }
    else
    {
        std::printf("[ OK ] %s: %.9f\n", name, actual);
    }
}

void expect_true(const char* name, bool cond)
{
    if (!cond)
    {
        std::printf("[FAIL] %s\n", name);
        ++g_failures;
    }
    else
    {
        std::printf("[ OK ] %s\n", name);
    }
}

// 用 FK 把 (yaw,pitch) 变成方向向量，与 desired_dir 比较夹角。
double fk_angle_error_deg(const AimResult& r)
{
    if (!r.reachable) return 999.0;  // 不可达不参与夹角比较
    const cv::Vec3d d_fk = aim_direction_from_angles(r.yaw, r.pitch);
    const double dot = std::clamp(d_fk.dot(r.desired_dir), -1.0, 1.0);
    return std::acos(dot) / kDeg;
}

// 与 aim_at_target 严格互逆的靶点生成。
cv::Vec3d target_from_angles(double yaw, double pitch, double range)
{
    return kLauncherPivotInBase + aim_direction_from_angles(yaw, pitch) * range;
}

// 绕 X 的旋转矩阵，供 plate_tilt 测试构造 R。
cv::Matx33d rot_x(double a)
{
    const double c = std::cos(a), s = std::sin(a);
    return {1, 0, 0, 0, c, -s, 0, s, c};
}

}  // namespace

int main()
{
    // 1. 正前方
    {
        const auto r = aim_at_target(target_from_angles(0, 0, 2.0));
        expect_true("straight_ahead reachable", r.reachable);
        expect_near("straight_ahead yaw", r.yaw, 0.0, 1e-9);
        expect_near("straight_ahead pitch", r.pitch, 0.0, 1e-9);
        expect_near("straight_ahead fk_err_deg", fk_angle_error_deg(r), 0.0, kAimTolDeg);
    }

    // 2. 零俯仰时，方向与 yaw 无关，反解约定返回 0。
    {
        const double yaw = 30 * kDeg;
        const auto r = aim_at_target(target_from_angles(yaw, 0, 2.0));
        expect_true("pure_yaw reachable", r.reachable);
        expect_near("pure_yaw yaw", r.yaw, 0.0, 1e-9);
        expect_near("pure_yaw pitch", r.pitch, 0.0, 1e-9);
        expect_near("pure_yaw fk_err_deg", fk_angle_error_deg(r), 0.0, kAimTolDeg);
    }

    // 3. 纯 pitch
    {
        const double pitch = 20 * kDeg;
        const auto r = aim_at_target(target_from_angles(0, pitch, 2.0));
        expect_true("pure_pitch reachable", r.reachable);
        expect_near("pure_pitch yaw", r.yaw, 0.0, 1e-9);
        expect_near("pure_pitch pitch", r.pitch, pitch, 1e-9);
        expect_near("pure_pitch fk_err_deg", fk_angle_error_deg(r), 0.0, kAimTolDeg);
    }

    // 4. 双轴组合
    {
        const double yaw = -25 * kDeg;
        const double pitch = 15 * kDeg;
        const auto r = aim_at_target(target_from_angles(yaw, pitch, 2.0));
        expect_true("combined reachable", r.reachable);
        expect_near("combined yaw", r.yaw, yaw, 1e-9);
        expect_near("combined pitch", r.pitch, pitch, 1e-9);
        expect_near("combined fk_err_deg", fk_angle_error_deg(r), 0.0, kAimTolDeg);
    }

    // 5. 负角度（pitch 在下限 -10° 内）
    {
        const double yaw = -45 * kDeg;
        const double pitch = -5 * kDeg;
        const auto r = aim_at_target(target_from_angles(yaw, pitch, 2.0));
        expect_true("negative reachable", r.reachable);
        expect_near("negative yaw", r.yaw, yaw, 1e-9);
        expect_near("negative pitch", r.pitch, pitch, 1e-9);
        expect_near("negative fk_err_deg", fk_angle_error_deg(r), 0.0, kAimTolDeg);
    }

    // 6. 超限：pitch 60° 超出上限 45°
    {
        const auto r = aim_at_target(target_from_angles(0, 60 * kDeg, 2.0));
        expect_true("out_of_range not reachable", !r.reachable);
    }

    // 7. 退化：靶点就在参考点上
    {
        const auto r = aim_at_target(kLauncherPivotInBase);
        expect_true("degenerate not reachable", !r.reachable);
    }

    // 8. Plate Tilt：正对靶 = 0°，绕 X 转 30° = 30°，法向取反不变
    {
        const double t0 = plate_tilt_rad(cv::Matx33d::eye()) / kDeg;
        expect_near("tilt_frontal_deg", t0, 0.0, 1e-9);

        const cv::Matx33d R = rot_x(30 * kDeg);
        const double t30 = plate_tilt_rad(R) / kDeg;
        expect_near("tilt_rotx30_deg", t30, 30.0, 1e-6);

        cv::Matx33d R_flip = R;
        R_flip(0, 2) = -R_flip(0, 2);
        R_flip(1, 2) = -R_flip(1, 2);
        R_flip(2, 2) = -R_flip(2, 2);
        expect_near("tilt_flip_invariant_deg", plate_tilt_rad(R_flip) / kDeg, 30.0, 1e-6);
    }

    // 9. normal_virtual_target：单位 R、法向 +Z 时虚拟靶点在参考点上方 range 处；取反不变
    {
        const cv::Vec3d n_cam{0.0, 0.0, 1.0};
        const cv::Vec3d vt = normal_virtual_target(n_cam, cv::Matx33d::eye(), 1.0);
        expect_near("normal_vt_x", vt[0], 0.0, 1e-9);
        expect_near("normal_vt_y", vt[1], 0.0, 1e-9);
        expect_near("normal_vt_z", vt[2], kLauncherPivotInBase[2] + 1.0, 1e-9);

        const cv::Vec3d vt_flip =
            normal_virtual_target(cv::Vec3d{0.0, 0.0, -1.0}, cv::Matx33d::eye(), 1.0);
        expect_near("normal_vt_flip_z", vt_flip[2], vt[2], 1e-9);
    }

    if (g_failures == 0)
    {
        std::printf("\nAll aim_at_target checks passed.\n");
        return 0;
    }
    std::printf("\n%d aim_at_target check(s) failed.\n", g_failures);
    return 1;
}
