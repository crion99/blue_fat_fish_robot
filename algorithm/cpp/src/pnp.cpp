#include "bff/pnp.hpp"

#include <cmath>
#include <limits>

namespace bff {
namespace {

// 单点重投影：返回像素预测值与雅可比 d(u,v)/d(δ, τ)，δ 为绕光心的旋转增量。
Vec2 projectPointJacobian(const Mat3& R, const Vec3& t, const Vec3& P, const Intrinsics& cam,
                          double J[2][6]) {
    const Vec3 p = R * P + t;
    double Jn[2][2];
    const Vec2 uv = cam.projectJacobian(p, Jn);

    const double z = std::fabs(p.z) < 1e-9 ? 1e-9 : p.z;
    const double invZ = 1.0 / z;
    const double x = p.x * invZ;
    const double y = p.y * invZ;

    // d(x,y)/d(p)：2x3
    const double dxy[2][3] = {{invZ, 0.0, -x * invZ}, {0.0, invZ, -y * invZ}};

    // A = Jn · dxy：2x3
    double A[2][3];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            A[i][j] = Jn[i][0] * dxy[0][j] + Jn[i][1] * dxy[1][j];

    // d(p)/dδ = -[p]x：3x3
    const double ns[3][3] = {{0.0, p.z, -p.y}, {-p.z, 0.0, p.x}, {p.y, -p.x, 0.0}};

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            J[i][j] = A[i][0] * ns[0][j] + A[i][1] * ns[1][j] + A[i][2] * ns[2][j];
            J[i][3 + j] = A[i][j];
        }
    }
    return uv;
}

struct RefineResult {
    Mat3 R{Mat3::identity()};
    Vec3 t{};
    double rms{std::numeric_limits<double>::infinity()};
    int iterations{0};
    bool finite{false};
};

RefineResult refine(const Mat3& R0, const Vec3& t0, const std::vector<Vec3>& obj,
                    const std::vector<Vec2>& uv, const Intrinsics& cam, int maxIters) {
    RefineResult out;
    out.R = R0;
    out.t = t0;
    const int n = static_cast<int>(obj.size());
    if (n < 4) return out;

    double lambda = 1e-6;
    for (int iter = 0; iter < maxIters; ++iter) {
        double H[6][6] = {};
        double g[6] = {};
        double cost = 0.0;
        bool ok = true;

        for (int i = 0; i < n; ++i) {
            double J[2][6];
            const Vec2 pred = projectPointJacobian(out.R, out.t, obj[i], cam, J);
            const double r0 = pred.x - uv[i].x;
            const double r1 = pred.y - uv[i].y;
            if (!std::isfinite(r0) || !std::isfinite(r1)) {
                ok = false;
                break;
            }
            cost += r0 * r0 + r1 * r1;
            for (int a = 0; a < 6; ++a) {
                g[a] += J[0][a] * r0 + J[1][a] * r1;
                for (int b = 0; b < 6; ++b) H[a][b] += J[0][a] * J[0][b] + J[1][a] * J[1][b];
            }
        }
        if (!ok) break;

        double Hd[6][6];
        for (int a = 0; a < 6; ++a) {
            for (int b = 0; b < 6; ++b) Hd[a][b] = H[a][b];
            Hd[a][a] += lambda * (H[a][a] + 1e-12);
        }
        double neg[6];
        for (int a = 0; a < 6; ++a) neg[a] = -g[a];
        double delta[6];
        if (!solveLinear<6>(Hd, neg, delta)) break;

        const Vec3 dTheta{delta[0], delta[1], delta[2]};
        const Mat3 dR = expSO3(dTheta);
        const Mat3 Rnew = dR * out.R;
        // 注意：δ 表示“相机绕光心旋转”，必须同时作用在 R 与 t 上。
        const Vec3 tnew = dR * out.t + Vec3{delta[3], delta[4], delta[5]};

        double costNew = 0.0;
        bool finite = true;
        for (int i = 0; i < n; ++i) {
            const Vec2 p = cam.project(Rnew * obj[i] + tnew);
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
                finite = false;
                break;
            }
            const double r0 = p.x - uv[i].x;
            const double r1 = p.y - uv[i].y;
            costNew += r0 * r0 + r1 * r1;
        }

        if (finite && costNew < cost) {
            out.R = Rnew;
            out.t = tnew;
            out.iterations = iter + 1;
            out.rms = std::sqrt(costNew / (2.0 * n));
            out.finite = true;
            lambda = std::max(lambda * 0.3, 1e-9);
            if (dTheta.norm() + Vec3{delta[3], delta[4], delta[5]}.norm() < 1e-10) break;
        } else {
            lambda *= 10.0;
            if (lambda > 1e6) break;
        }
    }

    if (!out.finite) {
        double s = 0.0;
        for (int i = 0; i < n; ++i) {
            const Vec2 p = cam.project(out.R * obj[i] + out.t);
            const double r0 = p.x - uv[i].x;
            const double r1 = p.y - uv[i].y;
            if (!std::isfinite(r0) || !std::isfinite(r1)) return out;
            s += r0 * r0 + r1 * r1;
        }
        out.rms = std::sqrt(s / (2.0 * n));
        out.finite = true;
    }
    return out;
}

// Hartley 归一化：去质心 + 平均距离缩放到 sqrt(2)
Mat3 normalizer(const std::vector<Vec2>& pts) {
    Vec2 c;
    for (const auto& p : pts) c += p;
    c = c * (1.0 / static_cast<double>(pts.size()));
    double meanDist = 0.0;
    for (const auto& p : pts) meanDist += (p - c).norm();
    meanDist /= static_cast<double>(pts.size());
    const double s = std::sqrt(2.0) / std::max(meanDist, 1e-12);
    Mat3 T = Mat3::identity();
    T.m[0][0] = s;
    T.m[1][1] = s;
    T.m[0][2] = -s * c.x;
    T.m[1][2] = -s * c.y;
    return T;
}

Vec2 applyH(const Mat3& H, const Vec2& p) {
    const double w = H.m[2][0] * p.x + H.m[2][1] * p.y + H.m[2][2];
    const double ww = std::fabs(w) < 1e-12 ? 1e-12 : w;
    return {(H.m[0][0] * p.x + H.m[0][1] * p.y + H.m[0][2]) / ww,
            (H.m[1][0] * p.x + H.m[1][1] * p.y + H.m[1][2]) / ww};
}

}  // namespace

Mat3 homographyDLT(const std::vector<Vec2>& objXY, const std::vector<Vec2>& normXY) {
    const size_t n = objXY.size();
    if (n < 4 || normXY.size() != n) return Mat3::identity();

    const Mat3 Tobj = normalizer(objXY);
    const Mat3 Timg = normalizer(normXY);

    // A^T A（9x9），按行累加
    double ATA[9][9] = {};
    for (size_t i = 0; i < n; ++i) {
        const Vec2 po = applyH(Tobj, objXY[i]);
        const Vec2 pi = applyH(Timg, normXY[i]);
        const double X = po.x, Y = po.y, x = pi.x, y = pi.y;
        const double row0[9] = {X, Y, 1.0, 0.0, 0.0, 0.0, -x * X, -x * Y, -x};
        const double row1[9] = {0.0, 0.0, 0.0, X, Y, 1.0, -y * X, -y * Y, -y};
        for (int a = 0; a < 9; ++a) {
            for (int b = 0; b < 9; ++b) ATA[a][b] += row0[a] * row0[b] + row1[a] * row1[b];
        }
    }

    const SymEigen<9> eig = symmetricEigen<9>(ATA);
    int minIdx = 0;
    for (int i = 1; i < 9; ++i)
        if (eig.values[i] < eig.values[minIdx]) minIdx = i;

    Mat3 Hn;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) Hn.m[r][c] = eig.vectors[3 * r + c][minIdx];

    Mat3 H = Timg.inverse() * Hn * Tobj;
    const double h22 = H.m[2][2];
    if (std::fabs(h22) > 1e-12) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) H.m[r][c] /= h22;
    }
    return H;
}

void poseFromHomography(const Mat3& H, Mat3& R, Vec3& t) {
    const Vec3 h1 = H.col(0);
    const Vec3 h2 = H.col(1);
    const Vec3 h3 = H.col(2);
    const double denom = h1.norm() + h2.norm();
    const double scale = denom > 1e-12 ? 2.0 / denom : 1.0;

    Vec3 r1 = h1 * scale;
    Vec3 r2 = h2 * scale;
    t = h3 * scale;
    if (t.z < 0.0) {
        r1 = -r1;
        r2 = -r2;
        t = -t;
    }
    // Gram-Schmidt
    r1 = r1.normalized();
    r2 = (r2 - r1 * r1.dot(r2)).normalized();
    const Vec3 r3 = cross(r1, r2);
    R = Mat3::fromColumns(r1, r2, r3);
}

double reprojectionRms(const Mat3& R, const Vec3& t, const std::vector<Vec3>& obj,
                       const std::vector<Vec2>& uv, const Intrinsics& cam) {
    double s = 0.0;
    for (size_t i = 0; i < obj.size(); ++i) {
        const Vec2 p = cam.project(R * obj[i] + t);
        const double dx = p.x - uv[i].x;
        const double dy = p.y - uv[i].y;
        s += dx * dx + dy * dy;
    }
    return std::sqrt(s / (2.0 * static_cast<double>(obj.size())));
}

PoseResult solvePnPPlanar(const std::vector<Vec3>& obj, const std::vector<Vec2>& uv,
                          const Intrinsics& cam, bool multiStart, int maxIters) {
    PoseResult result;
    const size_t n = obj.size();
    result.nPoints = static_cast<int>(n);
    if (n < 4 || uv.size() != n) return result;
    for (const auto& p : obj) {
        if (std::fabs(p.z) > 1e-6) return result;  // 只支持共面靶
    }

    std::vector<Vec2> objXY(n), normXY(n);
    for (size_t i = 0; i < n; ++i) {
        objXY[i] = {obj[i].x, obj[i].y};
        normXY[i] = cam.pixelToNormalized(uv[i]);
    }

    const Mat3 H = homographyDLT(objXY, normXY);
    Mat3 R0;
    Vec3 t0;
    poseFromHomography(H, R0, t0);

    std::vector<std::pair<Mat3, Vec3>> starts;
    starts.emplace_back(R0, t0);
    if (multiStart) {
        starts.emplace_back(R0 * rotX(kPi), t0);       // 镜像解
        starts.emplace_back(R0 * rotX(0.08), t0);
        starts.emplace_back(R0 * rotX(-0.08), t0);
        starts.emplace_back(R0 * rotY(0.08), t0);
        starts.emplace_back(R0 * rotY(-0.08), t0);
    }

    double bestScore = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < starts.size(); ++k) {
        const RefineResult rr = refine(starts[k].first, starts[k].second, obj, uv, cam, maxIters);
        if (!rr.finite) continue;

        bool allFront = true;
        for (size_t i = 0; i < n; ++i) {
            if ((rr.R * obj[i] + rr.t).z <= 0.0) {
                allFront = false;
                break;
            }
        }
        const double score = allFront ? rr.rms : rr.rms + 1e6;
        if (score < bestScore) {
            bestScore = score;
            result.R = rr.R;
            result.t = rr.t;
            result.reprojRmsPx = rr.rms;
            result.iterations = rr.iterations;
            result.startIndex = static_cast<int>(k);
            result.ok = allFront;
        }
    }
    return result;
}

}  // namespace bff
