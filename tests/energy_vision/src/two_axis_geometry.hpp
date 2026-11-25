#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <opencv2/core.hpp>

namespace energy_vision
{

struct RigidTransform
{
    cv::Matx33d rotation;
    cv::Vec3d translation;

    cv::Vec3d apply(const cv::Vec3d& point) const
    {
        return rotation * point + translation;
    }

    RigidTransform inverse() const
    {
        const auto inverse_rotation = rotation.t();
        return {inverse_rotation, -(inverse_rotation * translation)};
    }
};

// Compose A<-B with B<-C to obtain A<-C.
inline RigidTransform compose(const RigidTransform& outer, const RigidTransform& inner)
{
    return {outer.rotation * inner.rotation, outer.apply(inner.translation)};
}

inline std::optional<RigidTransform> make_rigid_transform(
    const cv::Matx33d& rotation, const cv::Vec3d& translation)
{
    for (double value : rotation.val)
        if (!std::isfinite(value))
            return std::nullopt;
    for (double value : translation.val)
        if (!std::isfinite(value))
            return std::nullopt;

    const auto orthogonality_error = rotation.t() * rotation - cv::Matx33d::eye();
    constexpr double tolerance = 1e-9;
    if (cv::norm(orthogonality_error, cv::NORM_INF) > tolerance ||
        std::abs(cv::determinant(rotation) - 1.0) > tolerance)
        return std::nullopt;
    return RigidTransform{rotation, translation};
}

struct JointRange
{
    double lower;
    double upper;

    bool contains(double angle) const
    {
        return std::isfinite(lower) && std::isfinite(upper) &&
            std::isfinite(angle) && angle >= lower && angle <= upper;
    }
};

inline cv::Matx33d rotation_x(double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {1, 0, 0, 0, c, -s, 0, s, c};
}

inline cv::Matx33d rotation_y(double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c, 0, s, 0, 1, 0, -s, 0, c};
}

inline cv::Matx33d rotation_z(double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c, -s, 0, s, c, 0, 0, 0, 1};
}

// Active rotations: parent yaw about +Z, then child pitch about its +X.
inline std::optional<cv::Matx33d> yaw_pitch_rotation(
    double yaw, double pitch, JointRange yaw_range, JointRange pitch_range)
{
    if (!yaw_range.contains(yaw) || !pitch_range.contains(pitch))
        return std::nullopt;

    return rotation_z(yaw) * rotation_x(pitch);
}

struct JointAngles
{
    double yaw;
    double pitch;
};

// Decompose a complete orientation. Limits must lie within the open principal
// interval so there is only one admissible representation of each angle.
inline std::optional<JointAngles> decompose_yaw_pitch_orientation(
    const cv::Matx33d& orientation, JointRange yaw_range, JointRange pitch_range)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double tolerance = 1e-9;
    const auto principal_range = [pi](JointRange range)
    {
        return range.lower > -pi && range.upper < pi && range.lower <= range.upper;
    };
    if (!principal_range(yaw_range) || !principal_range(pitch_range) ||
        !make_rigid_transform(orientation, {0, 0, 0}))
        return std::nullopt;

    const double yaw = std::atan2(orientation(1, 0), orientation(0, 0));
    const double pitch = std::atan2(orientation(2, 1), orientation(2, 2));
    if (yaw < yaw_range.lower - tolerance || yaw > yaw_range.upper + tolerance ||
        pitch < pitch_range.lower - tolerance || pitch > pitch_range.upper + tolerance)
        return std::nullopt;

    const JointAngles angles{
        std::clamp(yaw, yaw_range.lower, yaw_range.upper),
        std::clamp(pitch, pitch_range.lower, pitch_range.upper)
    };
    const auto reconstructed = rotation_z(angles.yaw) * rotation_x(angles.pitch);
    if (cv::norm(reconstructed - orientation, cv::NORM_INF) > tolerance)
        return std::nullopt;
    return angles;
}



}  // namespace energy_vision
