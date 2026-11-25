// 由靶标位姿解算云台瞄准转角。
//
// 链路：
//   相机系靶心 p_C
//     --(相机→云台外参 R_cg, t_cg)-->     当前云台头坐标系下的靶心 p_g
//     --(当前云台姿态 R_g(ψ_c,θ_c))-->     基座/世界系下的靶心 u = R_g·p_g
//     --(枪口偏移随云台转动)-->             d = u − R_g(ψ*,θ*)·t_muzzle
//     --> ψ* = atan2(d_y,d_x),  θ* = asin(d_z/|d|)
//     --> 转角 Δψ = ψ*−ψ_c, Δθ = θ*−θ_c
//
// 注意：相机随云台转动，p_g 只在“当前姿态”的云台头坐标系里成立，
// 因此必须先乘当前姿态再解算；反过来（把靶心当成基座系固定点）会重复叠加姿态，
// 导致误差随 yaw 线性增长——这是早期版本踩过的坑，已由闭环自检覆盖。

#pragma once

#include <string>

#include "bff/camera.hpp"
#include "bff/linalg.hpp"

namespace bff {

struct GimbalConfig {
    Mat3 R_cam_to_gimbal{Mat3::identity()};  // 相机系 -> 云台 zero 位系
    Vec3 t_cam_in_gimbal{};                  // 相机光心在云台 zero 位系的位置（m）
    Vec3 t_muzzle_in_gimbal{};               // 枪口在云台 zero 位系的位置（m）
    double yaw_min{-kPi};
    double yaw_max{kPi};
    double pitch_min{-0.5};
    double pitch_max{0.7};
    double gravity{9.81};
    double muzzle_speed{0.0};  // m/s，>0 时启用重力补偿
};

struct AimSolution {
    double yaw{0.0};          // 绝对 yaw（rad）
    double pitch{0.0};        // 绝对 pitch（rad）
    double deltaYaw{0.0};     // 相对当前朝向的转角（rad，已归一化）
    double deltaPitch{0.0};
    double distance{0.0};     // 枪口到靶心距离（m）
    double yawCurrent{0.0};
    double pitchCurrent{0.0};
    int iterations{0};
    double residualRad{0.0};
    bool clamped{false};
};

// 方向 -> (yaw, pitch)，与 gimbalRotation 严格配套
void anglesFromDirection(const Vec3& dir, double& yaw, double& pitch);

// (yaw, pitch) -> 枪管指向（世界系单位向量）
Vec3 directionFromAngles(double yaw, double pitch);

class AimSolver {
public:
    explicit AimSolver(const GimbalConfig& cfg = GimbalConfig{});

    const GimbalConfig& config() const { return cfg_; }

    // 相机系 -> 当前云台头坐标系
    Vec3 targetToGimbal(const Vec3& targetInCamera) const;

    // 解算瞄准角度
    AimSolution solve(const Vec3& targetInCamera, double yawCurrent, double pitchCurrent,
                      bool useBallistics = false) const;

    // 闭环自检：云台按指令转到 (yawCmd, pitchCmd) 后，枪管与靶心方向的角度偏差（rad）
    Vec2 aimError(const Vec3& targetInCamera, double yawCurrent, double pitchCurrent,
                  double yawCmd, double pitchCmd) const;

private:
    double ballisticPitch(double pitch, double distance) const;

    GimbalConfig cfg_;
};

}  // namespace bff
