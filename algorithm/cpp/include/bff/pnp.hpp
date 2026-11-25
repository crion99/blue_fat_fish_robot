// 平面靶 PnP：单应矩阵 DLT 求初值 + 高斯-牛顿最小化重投影误差。
//
// 输入 obj（靶标系 3D 点，必须共面，z≈0，N>=4）与 uv（对应像素坐标），
// 输出满足 p_cam = R·P_obj + t 的位姿。
//
// 平面靶在接近正视时有二义性（镜像解重投影误差相近），因此默认做多初值：
// DLT 解、镜像解、以及若干小角度扰动，最后取“点在相机前方 + 重投影误差最小”的解。

#pragma once

#include <vector>

#include "bff/camera.hpp"
#include "bff/linalg.hpp"

namespace bff {

struct PoseResult {
    Mat3 R{Mat3::identity()};
    Vec3 t{};
    double reprojRmsPx{1e9};
    int nPoints{0};
    int iterations{0};
    int startIndex{-1};
    bool ok{false};

    double distance() const { return t.norm(); }
};

// 单应矩阵（归一化 DLT，内部做 Hartley 归一化）
Mat3 homographyDLT(const std::vector<Vec2>& objXY, const std::vector<Vec2>& normXY);

// 由单应矩阵分解位姿（旋转用 Gram-Schmidt 正交化）
void poseFromHomography(const Mat3& H, Mat3& R, Vec3& t);

// 平面靶位姿解算
PoseResult solvePnPPlanar(const std::vector<Vec3>& obj, const std::vector<Vec2>& uv,
                          const Intrinsics& cam, bool multiStart = true, int maxIters = 40);

// 给定位姿重投影，返回均方根误差（像素），用于自检
double reprojectionRms(const Mat3& R, const Vec3& t, const std::vector<Vec3>& obj,
                       const std::vector<Vec2>& uv, const Intrinsics& cam);

}  // namespace bff
