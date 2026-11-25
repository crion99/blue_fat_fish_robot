// 位姿解算 + 瞄准转角的整合流程（第一部分检测到角点之后的全部计算）。

#pragma once

#include <array>
#include <string>

#include "bff/aim.hpp"
#include "bff/camera.hpp"
#include "bff/pnp.hpp"

namespace bff {

// 角点顺序：0 左上、1 右上、2 右下、3 左下（从相机看过去逆时针）。
// 该顺序是检测模块与本模块之间的硬约定。
struct TargetModel {
    double size{0.15};  // 靶面边长（m），须按规则手册修改
    std::string name{"rune_target"};

    std::array<Vec3, 4> points() const {
        const double h = size * 0.5;
        return {Vec3{-h, h, 0.0}, Vec3{h, h, 0.0}, Vec3{h, -h, 0.0}, Vec3{-h, -h, 0.0}};
    }
};

struct AimEstimate {
    PoseResult pose;
    AimSolution aim;
    bool accepted{false};
    std::string rejectReason;
};

// 一阶低通（指数滑动平均）：抑制逐帧抖动，转角跳变超过阈值时直接重置。
class AngleSmoother {
public:
    explicit AngleSmoother(double alpha = 0.4, double resetThresholdDeg = 5.0)
        : alpha_(alpha), resetThreshold_(resetThresholdDeg * kDeg2Rad) {}

    void reset() { init_ = false; }

    void update(double yaw, double pitch, double& outYaw, double& outPitch) {
        if (!init_) {
            yaw_ = yaw;
            pitch_ = pitch;
            init_ = true;
        } else if (std::fabs(wrapAngle(yaw - yaw_)) > resetThreshold_ ||
                   std::fabs(pitch - pitch_) > resetThreshold_) {
            yaw_ = yaw;
            pitch_ = pitch;
        } else {
            yaw_ = wrapAngle(yaw_ + alpha_ * wrapAngle(yaw - yaw_));
            pitch_ += alpha_ * (pitch - pitch_);
        }
        outYaw = yaw_;
        outPitch = pitch_;
    }

private:
    double alpha_{0.4};
    double resetThreshold_{5.0 * kDeg2Rad};
    double yaw_{0.0};
    double pitch_{0.0};
    bool init_{false};
};

class PoseEstimator {
public:
    PoseEstimator(const Intrinsics& cam, const TargetModel& target = TargetModel{},
                  const GimbalConfig& gimbal = GimbalConfig{}, double maxReprojRmsPx = 3.0)
        : cam_(cam), gimbal_(gimbal), target_(target), maxRms_(maxReprojRmsPx), solver_(gimbal) {}

    // 只解位姿
    PoseResult estimate(const std::array<Vec2, 4>& corners, bool multiStart = true) const;

    // 位姿 + 转角
    AimEstimate aim(const std::array<Vec2, 4>& corners, double yawCurrent, double pitchCurrent,
                    bool useBallistics = false) const;

    // 给电控的一行 ASCII 指令：AIM,<Δyaw_deg>,<Δpitch_deg>,<distance_m>,<OK|REJ>
    static std::string formatCommand(const AimEstimate& estimate, int decimals = 2);

    const AimSolver& solver() const { return solver_; }
    const TargetModel& target() const { return target_; }

private:
    Intrinsics cam_;
    GimbalConfig gimbal_;
    TargetModel target_;
    double maxRms_{3.0};
    AimSolver solver_;
};

}  // namespace bff
