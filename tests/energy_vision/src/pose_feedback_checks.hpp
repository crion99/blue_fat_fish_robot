#pragma once

#include <cmath>

namespace energy_vision
{

inline constexpr double radians_to_degrees(double radians)
{
    return radians * (180.0 / 3.14159265358979323846);
}

inline bool is_visible_camera_point(double x, double y, double z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0;
}

inline const char* feedback_status(
    double yaw, double pitch, double age_ms, double timeout_ms,
    double receipt_offset_ms, double max_offset_ms)
{
    if (!std::isfinite(yaw) || !std::isfinite(pitch))
        return "invalid_feedback";
    if (age_ms > timeout_ms)
        return "stale_feedback";

    // Joint position limits from assets/models/launcher/model.sdf, in radians.
    constexpr double tolerance = 1e-6;
    if (std::abs(yaw) > 1.570796 + tolerance ||
        pitch < -0.174533 - tolerance || pitch > 0.785398 + tolerance)
        return "feedback_outside_joint_limits";
    if (std::abs(receipt_offset_ms) > max_offset_ms)
        return "timestamp_mismatch";
    return "approximate";
}

}  // namespace energy_vision
