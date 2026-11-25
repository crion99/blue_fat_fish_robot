#include "bff/aim.hpp"

#include <algorithm>
#include <cmath>

namespace bff {

void anglesFromDirection(const Vec3& dir, double& yaw, double& pitch) {
    const Vec3 d = dir.normalized();
    if (d.norm() < 1e-12) {
        yaw = 0.0;
        pitch = 0.0;
        return;
    }
    pitch = std::asin(std::clamp(d.z, -1.0, 1.0));
    yaw = std::atan2(d.y, d.x);
}

Vec3 directionFromAngles(double yaw, double pitch) {
    return {std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)};
}

AimSolver::AimSolver(const GimbalConfig& cfg) : cfg_(cfg) {}

Vec3 AimSolver::targetToGimbal(const Vec3& targetInCamera) const {
    return cfg_.R_cam_to_gimbal * targetInCamera + cfg_.t_cam_in_gimbal;
}

AimSolution AimSolver::solve(const Vec3& targetInCamera, double yawCurrent, double pitchCurrent,
                             bool useBallistics) const {
    AimSolution out;
    const Vec3 pG = targetToGimbal(targetInCamera);
    const Vec3 uWorld = gimbalRotation(yawCurrent, pitchCurrent) * pG;

    double yaw = yawCurrent;
    double pitch = pitchCurrent;
    double residual = 1e9;
    Mat3 Rg = gimbalRotation(yaw, pitch);

    constexpr int kMaxIter = 8;
    for (int i = 0; i < kMaxIter; ++i) {
        Rg = gimbalRotation(yaw, pitch);
        const Vec3 d = uWorld - Rg * cfg_.t_muzzle_in_gimbal;
        double yawNew = 0.0, pitchNew = 0.0;
        anglesFromDirection(d, yawNew, pitchNew);
        residual = std::max(std::fabs(wrapAngle(yawNew - yaw)), std::fabs(pitchNew - pitch));
        yaw = yawNew;
        pitch = pitchNew;
        out.iterations = i + 1;
        if (residual < 1e-9) break;
    }

    const double distance = (uWorld - Rg * cfg_.t_muzzle_in_gimbal).norm();
    if (useBallistics) pitch = ballisticPitch(pitch, distance);

    const double yawClamped = std::clamp(yaw, cfg_.yaw_min, cfg_.yaw_max);
    const double pitchClamped = std::clamp(pitch, cfg_.pitch_min, cfg_.pitch_max);
    out.clamped = std::fabs(yawClamped - yaw) > 1e-12 || std::fabs(pitchClamped - pitch) > 1e-12;

    out.yaw = yaw;
    out.pitch = pitch;
    out.deltaYaw = wrapAngle(yaw - yawCurrent);
    out.deltaPitch = pitch - pitchCurrent;
    out.distance = distance;
    out.yawCurrent = yawCurrent;
    out.pitchCurrent = pitchCurrent;
    out.residualRad = residual;
    return out;
}

double AimSolver::ballisticPitch(double pitch, double distance) const {
    // 平抛近似：水平距离 d·cosθ，飞行时间 t = d·cosθ/(v·cosθ)，落差 ½gt²，补偿角 atan(落差/水平距离)
    const double v0 = cfg_.muzzle_speed;
    if (v0 <= 1e-6) return pitch;
    double p = pitch;
    for (int i = 0; i < 2; ++i) {
        const double cosP = std::max(std::cos(p), 1e-3);
        const double dHoriz = std::max(distance * cosP, 1e-3);
        const double tFlight = dHoriz / (v0 * cosP);
        const double drop = 0.5 * cfg_.gravity * tFlight * tFlight;
        p = pitch + std::atan2(drop, dHoriz);
    }
    return std::min(p, cfg_.pitch_max);
}

Vec2 AimSolver::aimError(const Vec3& targetInCamera, double yawCurrent, double pitchCurrent,
                         double yawCmd, double pitchCmd) const {
    const Vec3 pG = targetToGimbal(targetInCamera);
    const Vec3 uWorld = gimbalRotation(yawCurrent, pitchCurrent) * pG;
    const Vec3 d = uWorld - gimbalRotation(yawCmd, pitchCmd) * cfg_.t_muzzle_in_gimbal;
    double yawT = 0.0, pitchT = 0.0;
    anglesFromDirection(d, yawT, pitchT);
    return {wrapAngle(yawT - yawCmd), pitchT - pitchCmd};
}

}  // namespace bff
