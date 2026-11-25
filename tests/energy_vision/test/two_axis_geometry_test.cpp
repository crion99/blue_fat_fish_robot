#include <iostream>
#include <limits>
#include <stdexcept>

#include "two_axis_geometry.hpp"

namespace
{

void check(bool condition, const char* description)
{
    if (!condition)
        throw std::runtime_error(description);
}

bool near(const cv::Vec3d& actual, const cv::Vec3d& expected)
{
    return cv::norm(actual - expected) < 1e-9;
}

}  // namespace

int main()
{
    using namespace energy_vision;
    try
    {
        constexpr double half_pi = 1.5707963267948966;
        constexpr JointRange range{-half_pi, half_pi};
        const cv::Vec3d x{1, 0, 0};
        const cv::Vec3d z{0, 0, 1};
        const double nan = std::numeric_limits<double>::quiet_NaN();

        check(near(yaw_pitch_rotation(0, 0, range, range).value() * z, z), "zero rotation");
        check(near(rotation_y(half_pi) * z, x), "positive rotation about Y");
        check(near(yaw_pitch_rotation(half_pi, 0, range, range).value() * x, {0, 1, 0}),
            "positive yaw");
        check(near(yaw_pitch_rotation(-half_pi, 0, range, range).value() * x, {0, -1, 0}),
            "negative yaw");
        check(near(yaw_pitch_rotation(0, half_pi, range, range).value() * z, {0, -1, 0}),
            "positive pitch");
        check(near(yaw_pitch_rotation(0, -half_pi, range, range).value() * z, {0, 1, 0}),
            "negative pitch");

        const auto combined = yaw_pitch_rotation(half_pi, half_pi, range, range).value();
        check(near(combined * z, x), "nonzero rotation order");
        const auto outer = make_rigid_transform(combined, {2, 3, 4}).value();
        const auto inner = make_rigid_transform(cv::Matx33d::eye(), {0, 0, 2}).value();
        check(near(outer.apply(z), {3, 3, 4}), "rotation plus translation");
        check(near(compose(outer, inner).apply(z), {5, 3, 4}), "frame composition");
        check(near(outer.inverse().apply(outer.apply(z)), z), "inverse round trip");

        constexpr JointRange sdf_yaw{-1.570796, 1.570796};
        constexpr JointRange sdf_pitch{-0.174533, 0.785398};
        check(yaw_pitch_rotation(sdf_yaw.lower, sdf_pitch.upper, sdf_yaw, sdf_pitch).has_value(),
            "inclusive joint limits");
        check(!yaw_pitch_rotation(1.6, 0, sdf_yaw, sdf_pitch), "yaw beyond limit");
        check(!yaw_pitch_rotation(0, -0.2, sdf_yaw, sdf_pitch), "pitch beyond limit");
        check(!yaw_pitch_rotation(0, nan, range, range), "non-finite angle");
        check(!yaw_pitch_rotation(0, 0, {1, -1}, range), "reversed limits");
        check(!make_rigid_transform(cv::Matx33d::eye(), {0, nan, 0}), "non-finite translation");
        check(!make_rigid_transform(cv::Matx33d::zeros(), {0, 0, 0}), "singular rotation");
        check(!make_rigid_transform(cv::Matx33d::diag({1, 1, -1}), {0, 0, 0}), "reflection");

        const auto zero = decompose_yaw_pitch_orientation(
            cv::Matx33d::eye(), sdf_yaw, sdf_pitch).value();
        check(zero.yaw == 0 && zero.pitch == 0, "zero orientation decomposition");

        const JointAngles current{0.23, -0.10};
        const JointAngles expected{-0.71, 0.46};
        const auto orientation =
            yaw_pitch_rotation(expected.yaw, expected.pitch, sdf_yaw, sdf_pitch).value();
        const auto angles = decompose_yaw_pitch_orientation(
            orientation, sdf_yaw, sdf_pitch).value();
        const double delta_yaw = angles.yaw - current.yaw;
        const double delta_pitch = angles.pitch - current.pitch;
        check(std::abs(delta_yaw + 0.94) < 1e-12, "yaw difference from nonzero state");
        check(std::abs(delta_pitch - 0.56) < 1e-12, "pitch difference from nonzero state");
        const auto reconstructed = yaw_pitch_rotation(
            current.yaw + delta_yaw, current.pitch + delta_pitch, sdf_yaw, sdf_pitch).value();
        const double matrix_error = cv::norm(reconstructed - orientation, cv::NORM_INF);
        check(matrix_error < 1e-12, "forward and inverse reconstruction");

        const auto boundary = decompose_yaw_pitch_orientation(
            rotation_z(sdf_yaw.lower) * rotation_x(sdf_pitch.upper), sdf_yaw, sdf_pitch);
        check(boundary.has_value(), "orientation at joint limits");
        check(!decompose_yaw_pitch_orientation(rotation_z(1.8), sdf_yaw, sdf_pitch),
            "unreachable yaw");
        check(!decompose_yaw_pitch_orientation(rotation_x(-0.3), sdf_yaw, sdf_pitch),
            "unreachable pitch");
        check(!decompose_yaw_pitch_orientation(rotation_y(0.2), sdf_yaw, sdf_pitch),
            "orientation requires a third axis");
        auto invalid = cv::Matx33d::eye();
        invalid(0, 0) = nan;
        check(!decompose_yaw_pitch_orientation(invalid, sdf_yaw, sdf_pitch),
            "non-finite orientation");
        check(!decompose_yaw_pitch_orientation(cv::Matx33d::eye(), {1, -1}, sdf_pitch),
            "invalid orientation limits");
        std::cout << "Current yaw/pitch (rad): " << current.yaw << ", " << current.pitch
                  << "\nRecovered yaw/pitch (rad): " << angles.yaw << ", " << angles.pitch
                  << "\nJoint differences (rad): " << delta_yaw << ", " << delta_pitch
                  << "\nMaximum absolute rotation matrix error: " << matrix_error << '\n';
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
