#include "bff/pipeline.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

namespace bff {

PoseResult PoseEstimator::estimate(const std::array<Vec2, 4>& corners, bool multiStart) const {
    const std::array<Vec3, 4> obj = target_.points();
    const std::vector<Vec3> objVec(obj.begin(), obj.end());
    const std::vector<Vec2> uv(corners.begin(), corners.end());
    return solvePnPPlanar(objVec, uv, cam_, multiStart);
}

AimEstimate PoseEstimator::aim(const std::array<Vec2, 4>& corners, double yawCurrent,
                               double pitchCurrent, bool useBallistics) const {
    AimEstimate out;
    out.pose = estimate(corners);

    if (!out.pose.ok) {
        out.accepted = false;
        out.rejectReason = "pose_failed";
    } else if (out.pose.reprojRmsPx > maxRms_) {
        out.accepted = false;
        out.rejectReason = "reprojection_too_large";
    } else {
        out.accepted = true;
    }

    out.aim = solver_.solve(out.pose.t, yawCurrent, pitchCurrent, useBallistics);
    return out;
}

std::string PoseEstimator::formatCommand(const AimEstimate& estimate, int decimals) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "AIM,%.*f,%.*f,%.3f,%s", decimals,
                  estimate.aim.deltaYaw * kRad2Deg, decimals,
                  estimate.aim.deltaPitch * kRad2Deg, estimate.aim.distance,
                  estimate.accepted ? "OK" : "REJ");
    return std::string(buf);
}

}  // namespace bff
