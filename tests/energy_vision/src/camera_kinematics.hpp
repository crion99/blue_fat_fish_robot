
#pragma once

#include "two_axis_geometry.hpp"

#include <cmath>
#include <optional>

namespace energy_vision
{

// Camera mounting transforms.
//
// Convention:
// A_from_B means the transform that converts
// coordinates expressed in frame B into frame A.
struct CameraMount
{
    // base_link <- yaw_joint
    RigidTransform base_from_yaw_joint;

    // yaw_joint <- yaw_link
    RigidTransform yaw_joint_from_yaw_link;

    // yaw_link <- pitch_joint
    RigidTransform yaw_from_pitch_joint;

    // pitch_joint <- pitch_link
    RigidTransform pitch_joint_from_pitch_link;

    // pitch_link <- camera frame
    RigidTransform pitch_from_camera;
};


// Construct a pure translation transform.
inline RigidTransform translation_transform(
    const cv::Vec3d& translation)
{
    return {
        cv::Matx33d::eye(),
        translation
    };
}


// ROBOMINI Launcher camera mounting parameters.
inline CameraMount robomini_camera_mount()
{
    constexpr double camera_yaw = 1.570796;

    return {

        // base_link <- yaw_joint
        translation_transform({
            0.0, 0.0, 0.080
        }),

        // yaw_joint <- yaw_link
        translation_transform({
            0.0, 0.0, -0.040
        }),

        // yaw_link <- pitch_joint
        translation_transform({
            0.0, 0.0, 0.290
        }),

        // pitch_joint <- pitch_link
        translation_transform({
            0.0, 0.0, -0.145
        }),

        // pitch_link <- camera
        {
            rotation_z(camera_yaw),
            cv::Vec3d{
                0.0, 0.036, 0.075
            }
        }
    };
}


// ROBOMINI Launcher joint limits.
inline JointRange robomini_yaw_range()
{
    return {
        -1.570796,
         1.570796
    };
}


inline JointRange robomini_pitch_range()
{
    return {
        -0.174533,
         0.785398
    };
}


// Forward kinematics:
//
// Input:
//   yaw_rad   - Yaw joint angle, radians
//   pitch_rad - Pitch joint angle, radians
//   mount     - Camera mounting parameters
//
// Output:
//   base_link <- camera transform
//
inline RigidTransform camera_pose_in_base(
    double yaw_rad,
    double pitch_rad,
    const CameraMount& mount)
{
    // 1. Yaw joint motion.
    const RigidTransform yaw_motion{
        rotation_z(yaw_rad),
        cv::Vec3d{0.0, 0.0, 0.0}
    };

    // base_link <- yaw_joint
    const auto base_from_yaw_joint =
        compose(
            mount.base_from_yaw_joint,
            yaw_motion
        );

    // base_link <- yaw_link
    const auto base_from_yaw_link =
        compose(
            base_from_yaw_joint,
            mount.yaw_joint_from_yaw_link
        );


    // 2. Pitch joint motion.
    const RigidTransform pitch_motion{
        rotation_x(pitch_rad),
        cv::Vec3d{0.0, 0.0, 0.0}
    };

    // yaw_link <- pitch_joint
    const auto yaw_from_pitch_joint =
        compose(
            mount.yaw_from_pitch_joint,
            pitch_motion
        );

    // yaw_link <- pitch_link
    const auto yaw_from_pitch_link =
        compose(
            yaw_from_pitch_joint,
            mount.pitch_joint_from_pitch_link
        );


    // 3. Combine Yaw and Pitch.
    // base_link <- pitch_link
    const auto base_from_pitch_link =
        compose(
            base_from_yaw_link,
            yaw_from_pitch_link
        );


    // 4. Apply fixed camera mounting transform.
    // base_link <- camera
    return compose(
        base_from_pitch_link,
        mount.pitch_from_camera
    );
}


// Validate all mounting transforms.
inline bool valid_camera_mount(
    const CameraMount& mount)
{
    return
        make_rigid_transform(
            mount.base_from_yaw_joint.rotation,
            mount.base_from_yaw_joint.translation
        ).has_value() &&

        make_rigid_transform(
            mount.yaw_joint_from_yaw_link.rotation,
            mount.yaw_joint_from_yaw_link.translation
        ).has_value() &&

        make_rigid_transform(
            mount.yaw_from_pitch_joint.rotation,
            mount.yaw_from_pitch_joint.translation
        ).has_value() &&

        make_rigid_transform(
            mount.pitch_joint_from_pitch_link.rotation,
            mount.pitch_joint_from_pitch_link.translation
        ).has_value() &&

        make_rigid_transform(
            mount.pitch_from_camera.rotation,
            mount.pitch_from_camera.translation
        ).has_value();
}


// Checked FK:
// Reject invalid angles, out-of-range joints,
// and invalid mounting transforms.
inline std::optional<RigidTransform>
camera_pose_in_base_checked(
    double yaw_rad,
    double pitch_rad,
    const CameraMount& mount,
    JointRange yaw_range,
    JointRange pitch_range)
{
    if (!yaw_range.contains(yaw_rad) ||
        !pitch_range.contains(pitch_rad))
    {
        return std::nullopt;
    }

    if (!valid_camera_mount(mount))
    {
        return std::nullopt;
    }

    return camera_pose_in_base(
        yaw_rad,
        pitch_rad,
        mount
    );
}


// Convenient ROBOMINI FK interface.
inline std::optional<RigidTransform>
robomini_camera_pose_in_base(
    double yaw_rad,
    double pitch_rad)
{
    const auto mount = robomini_camera_mount();

    return camera_pose_in_base_checked(
        yaw_rad,
        pitch_rad,
        mount,
        robomini_yaw_range(),
        robomini_pitch_range()
    );
}

}  // namespace energy_vision
