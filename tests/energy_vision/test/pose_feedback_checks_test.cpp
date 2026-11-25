#include <array>
#include <iostream>
#include <limits>
#include <string_view>

#include "pose_feedback_checks.hpp"

int main()
{
    using energy_vision::feedback_status;
    using energy_vision::is_visible_camera_point;
    using energy_vision::radians_to_degrees;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    if (radians_to_degrees(0.0) != 0.0 ||
        std::abs(radians_to_degrees(-1.5707963267948966) + 90.0) > 1e-9 ||
        std::abs(radians_to_degrees(0.7853981633974483) - 45.0) > 1e-9)
    {
        std::cerr << "Incorrect conversion of measured joint angles\n";
        return 1;
    }

    for (const auto& point : {
        std::array<double, 3>{0.0, 0.0, 2.0}, {-0.3, 0.0, 2.0}, {0.3, 0.0, 2.0},
        {0.0, -0.3, 2.0}, {0.0, 0.3, 2.0}, {0.292, -0.171, 1.913}})
    {
        if (!is_visible_camera_point(point[0], point[1], point[2]))
        {
            std::cerr << "Rejected a finite point in front of the camera\n";
            return 1;
        }
    }

    if (is_visible_camera_point(nan, 0.0, 1.0) ||
        is_visible_camera_point(0.0, 0.0, 0.0) ||
        is_visible_camera_point(0.0, 0.0, -1.0))
    {
        std::cerr << "Accepted invalid camera geometry\n";
        return 1;
    }

    using namespace std::literals;
    if (feedback_status(1.570796, -0.174533, 250, 250, -50, 50) != "approximate"sv ||
        feedback_status(nan, 0.0, 0, 250, 0, 50) != "invalid_feedback"sv ||
        feedback_status(0.0, 0.0, 251, 250, 0, 50) != "stale_feedback"sv ||
        feedback_status(1.6, 0.0, 0, 250, 0, 50) != "feedback_outside_joint_limits"sv ||
        feedback_status(0.0, 0.8, 0, 250, 0, 50) != "feedback_outside_joint_limits"sv ||
        feedback_status(0.0, 0.0, 0, 250, -51, 50) != "timestamp_mismatch"sv)
    {
        std::cerr << "Unexpected feedback validation status\n";
        return 1;
    }
}
