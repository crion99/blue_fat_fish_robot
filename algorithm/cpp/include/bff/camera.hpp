// 针孔相机 + Brown-Conrady 畸变（5 参数，与 OpenCV 标定输出一致）。
//
// 相机系 {C}：x 向右、y 向下、z 沿光轴向前（OpenCV 习惯）。
// 像素系：u 向右、v 向下，原点在左上角。

#pragma once

#include <cmath>
#include <string>

#include "bff/linalg.hpp"

namespace bff {

struct Intrinsics {
    double fx{1000.0};
    double fy{1000.0};
    double cx{640.0};
    double cy{360.0};
    int width{1280};
    int height{720};
    double k1{0.0};
    double k2{0.0};
    double p1{0.0};
    double p2{0.0};
    double k3{0.0};
    std::string name{"camera"};

    bool hasDistortion() const {
        return std::fabs(k1) + std::fabs(k2) + std::fabs(p1) + std::fabs(p2) + std::fabs(k3) >
               1e-12;
    }

    double hfovDeg() const { return 2.0 * std::atan(width / (2.0 * fx)) * kRad2Deg; }
    double vfovDeg() const { return 2.0 * std::atan(height / (2.0 * fy)) * kRad2Deg; }

    double radial(double r2) const { return 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3)); }

    // 理想归一化像点 -> 含畸变归一化像点
    Vec2 distort(const Vec2& xy) const {
        const double x = xy.x, y = xy.y;
        const double r2 = x * x + y * y;
        const double k = radial(r2);
        return {x * k + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                y * k + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y};
    }

    // 含畸变归一化像点 -> 理想归一化像点（不动点迭代）
    Vec2 undistort(const Vec2& xy_d, int iters = 8) const {
        double x = xy_d.x, y = xy_d.y;
        for (int i = 0; i < iters; ++i) {
            const double r2 = x * x + y * y;
            const double k = radial(r2);
            const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (xy_d.x - dx) / k;
            y = (xy_d.y - dy) / k;
        }
        return {x, y};
    }

    Vec2 pixelToNormalized(const Vec2& uv) const {
        const Vec2 xy_d{(uv.x - cx) / fx, (uv.y - cy) / fy};
        return hasDistortion() ? undistort(xy_d) : xy_d;
    }

    // 相机系 3D 点 -> 像素
    Vec2 project(const Vec3& p) const {
        const double z = std::fabs(p.z) < 1e-9 ? 1e-9 : p.z;
        const Vec2 xy{p.x / z, p.y / z};
        const Vec2 d = hasDistortion() ? distort(xy) : xy;
        return {fx * d.x + cx, fy * d.y + cy};
    }

    // 投影并返回对归一化像点 (x, y) 的雅可比，已含 fx/fy：J = d(u,v)/d(x,y)
    Vec2 projectJacobian(const Vec3& p, double J[2][2]) const {
        const double z = std::fabs(p.z) < 1e-9 ? 1e-9 : p.z;
        const double x = p.x / z, y = p.y / z;
        const double r2 = x * x + y * y;
        const double k = radial(r2);
        const double dk = 2.0 * (k1 + r2 * (2.0 * k2 + 3.0 * k3 * r2));
        const double dkdx = dk * x;
        const double dkdy = dk * y;

        const double xd = x * k + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double yd = y * k + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

        const double jxx = k + x * dkdx + 2.0 * p1 * y + 6.0 * p2 * x;
        const double jxy = x * dkdy + 2.0 * p1 * x + 2.0 * p2 * y;
        const double jyx = y * dkdx + 2.0 * p1 * x + 2.0 * p2 * y;
        const double jyy = k + y * dkdy + 6.0 * p1 * y + 2.0 * p2 * x;

        J[0][0] = fx * jxx;
        J[0][1] = fx * jxy;
        J[1][0] = fy * jyx;
        J[1][1] = fy * jyy;
        return {fx * xd + cx, fy * yd + cy};
    }

    // 像素 -> 相机系单位方向
    Vec3 ray(const Vec2& uv) const {
        const Vec2 xy = pixelToNormalized(uv);
        return Vec3{xy.x, xy.y, 1.0}.normalized();
    }
};

}  // namespace bff
