// 蓝胖鱼机器人 · 算法方向
// 小型线性代数库：只依赖标准 C++17，不引 Eigen / OpenCV。
//
// 约定：右手系、列向量、弧度制。
//   云台角：yaw 绕 +z（向左为正），pitch 抬头为正，对应 Rz(yaw)·Ry(-pitch)。
//   因此枪管轴线 e1 在姿态 (yaw, pitch) 下为 [cosψcosθ, sinψcosθ, sinθ]。

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace bff {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

// ---------------------------------------------------------------- Vec2 / Vec3

struct Vec2 {
    double x{0.0};
    double y{0.0};

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double norm() const { return std::sqrt(x * x + y * y); }
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        const double n = norm();
        return n > 1e-12 ? Vec3{x / n, y / n, z / n} : Vec3{};
    }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

inline double dot(const Vec3& a, const Vec3& b) { return a.dot(b); }

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a * (1.0 - t) + b * t; }

// ---------------------------------------------------------------------- Mat3

struct Mat3 {
    double m[3][3]{};

    Mat3() = default;

    static Mat3 identity() {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }

    static Mat3 fromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            r.m[i][0] = c0[i];
            r.m[i][1] = c1[i];
            r.m[i][2] = c2[i];
        }
        return r;
    }

    Vec3 col(int j) const { return {m[0][j], m[1][j], m[2][j]}; }

    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }

    Mat3 transpose() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }

    double det() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    Mat3 inverse() const {
        const double d = det();
        Mat3 r;
        if (std::fabs(d) < 1e-15) return Mat3::identity();
        const double inv = 1.0 / d;
        r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
        r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
        r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
        r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
        r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
        r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
        r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
        r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
        r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
        return r;
    }
};

inline Mat3 rotX(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r = Mat3::identity();
    r.m[1][1] = c;
    r.m[1][2] = -s;
    r.m[2][1] = s;
    r.m[2][2] = c;
    return r;
}

inline Mat3 rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r = Mat3::identity();
    r.m[0][0] = c;
    r.m[0][2] = s;
    r.m[2][0] = -s;
    r.m[2][2] = c;
    return r;
}

inline Mat3 rotZ(double a) {
    const double c = std::cos(a), s = std::sin(a);
    Mat3 r = Mat3::identity();
    r.m[0][0] = c;
    r.m[0][1] = -s;
    r.m[1][0] = s;
    r.m[1][1] = c;
    return r;
}

// 云台 zero 位 -> 世界/车体系；枪管 e1 指向 [cosψcosθ, sinψcosθ, sinθ]
inline Mat3 gimbalRotation(double yaw, double pitch) {
    return rotZ(yaw) * rotY(-pitch);
}

// ------------------------------------------------------------- SO(3) 指数/对数

inline Mat3 expSO3(const Vec3& w) {
    const double theta = w.norm();
    const double t2 = theta * theta;
    // 小角度用泰勒展开的一阶形式，避免除零
    if (theta < 1e-12) {
        Mat3 r = Mat3::identity();
        r.m[0][1] = -w.z;
        r.m[0][2] = w.y;
        r.m[1][0] = w.z;
        r.m[1][2] = -w.x;
        r.m[2][0] = -w.y;
        r.m[2][1] = w.x;
        return r;
    }
    const Vec3 k = w * (1.0 / theta);
    const Mat3 K = Mat3::fromColumns(cross(k, Vec3{1, 0, 0}), cross(k, Vec3{0, 1, 0}),
                                     cross(k, Vec3{0, 0, 1}));
    // K = [k]x，上式逐列等价于 skew(k)
    const Mat3 K2 = K * K;
    Mat3 r = Mat3::identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] += std::sin(theta) * K.m[i][j] + (1.0 - std::cos(theta)) * K2.m[i][j];
    (void)t2;
    return r;
}

inline Vec3 logSO3(const Mat3& R) {
    const double c = std::clamp((R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0) * 0.5, -1.0, 1.0);
    const double theta = std::acos(c);
    if (theta < 1e-9) {
        return {R.m[2][1] - R.m[1][2], R.m[0][2] - R.m[2][0], R.m[1][0] - R.m[0][1]};
    }
    if (std::fabs(kPi - theta) < 1e-6) {
        // 近 180°：由 (R + I)/2 = k k^T 取轴
        const double A[3][3] = {{(R.m[0][0] + 1) * 0.5, R.m[0][1] * 0.5, R.m[0][2] * 0.5},
                                {R.m[1][0] * 0.5, (R.m[1][1] + 1) * 0.5, R.m[1][2] * 0.5},
                                {R.m[2][0] * 0.5, R.m[2][1] * 0.5, (R.m[2][2] + 1) * 0.5}};
        int idx = 0;
        for (int i = 1; i < 3; ++i)
            if (A[i][i] > A[idx][idx]) idx = i;
        const double s = std::sqrt(std::max(A[idx][idx], 1e-12));
        return Vec3{A[0][idx] / s, A[1][idx] / s, A[2][idx] / s} * theta;
    }
    const double k = theta / (2.0 * std::sin(theta));
    return {k * (R.m[2][1] - R.m[1][2]), k * (R.m[0][2] - R.m[2][0]), k * (R.m[1][0] - R.m[0][1])};
}

inline double wrapAngle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a < 0.0) a += 2.0 * kPi;
    return a - kPi;
}

inline double rotationAngleDeg(const Mat3& a, const Mat3& b) {
    const Mat3 d = a.transpose() * b;
    const double c = std::clamp((d.m[0][0] + d.m[1][1] + d.m[2][2] - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(c) * kRad2Deg;
}

// ------------------------------------------------------- N×N 线性方程组与特征值

// 部分主元高斯消元；A、b 按值传入，返回是否可解。
template <int N>
bool solveLinear(const double Ain[N][N], const double bin[N], double xout[N]) {
    double A[N][N];
    double b[N];
    for (int i = 0; i < N; ++i) {
        b[i] = bin[i];
        for (int j = 0; j < N; ++j) A[i][j] = Ain[i][j];
    }
    for (int col = 0; col < N; ++col) {
        int pivot = col;
        for (int r = col + 1; r < N; ++r)
            if (std::fabs(A[r][col]) > std::fabs(A[pivot][col])) pivot = r;
        if (std::fabs(A[pivot][col]) < 1e-14) return false;
        if (pivot != col) {
            for (int j = 0; j < N; ++j) std::swap(A[col][j], A[pivot][j]);
            std::swap(b[col], b[pivot]);
        }
        const double inv = 1.0 / A[col][col];
        for (int r = col + 1; r < N; ++r) {
            const double f = A[r][col] * inv;
            if (f == 0.0) continue;
            for (int j = col; j < N; ++j) A[r][j] -= f * A[col][j];
            b[r] -= f * b[col];
        }
    }
    for (int i = N - 1; i >= 0; --i) {
        double s = b[i];
        for (int j = i + 1; j < N; ++j) s -= A[i][j] * xout[j];
        xout[i] = s / A[i][i];
    }
    return true;
}

template <int N>
struct SymEigen {
    double values[N]{};
    double vectors[N][N]{};  // 第 j 列是第 j 个特征向量
};

// 对称矩阵的 Jacobi 特征分解（N 最大 9，够 DLT/位姿使用）。
template <int N>
SymEigen<N> symmetricEigen(const double Ain[N][N], int max_sweeps = 60) {
    double A[N][N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) A[i][j] = Ain[i][j];

    SymEigen<N> out;
    for (int i = 0; i < N; ++i) out.vectors[i][i] = 1.0;

    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < N; ++p)
            for (int q = p + 1; q < N; ++q) off += A[p][q] * A[p][q];
        if (off < 1e-26) break;

        for (int p = 0; p < N; ++p) {
            for (int q = p + 1; q < N; ++q) {
                if (std::fabs(A[p][q]) < 1e-18) continue;
                const double theta = (A[q][q] - A[p][p]) / (2.0 * A[p][q]);
                const double sign = theta >= 0.0 ? 1.0 : -1.0;
                const double t = sign / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;

                for (int k = 0; k < N; ++k) {
                    const double akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < N; ++k) {
                    const double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < N; ++k) {
                    const double vkp = out.vectors[k][p], vkq = out.vectors[k][q];
                    out.vectors[k][p] = c * vkp - s * vkq;
                    out.vectors[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    for (int i = 0; i < N; ++i) out.values[i] = A[i][i];
    return out;
}

}  // namespace bff
