// 第三部分自检：合成数据蒙特卡洛。
//
// 行为：随机生成靶标位姿 -> 投影成带畸变像素 -> 加高斯噪声 -> 解算位姿与转角 ->
//       与几何真值比对，输出控制台表格与 tests/ 下的报告。
//
// 用法：pose_selftest [输出目录] [每组试验次数]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "bff/aim.hpp"
#include "bff/camera.hpp"
#include "bff/pipeline.hpp"
#include "bff/pnp.hpp"

using namespace bff;

namespace {

// 与 configs/*.yml 保持一致的默认参数（改配置时同步改这里，或由 OpenCV 版程序读配置）
Intrinsics defaultCamera() {
    Intrinsics c;
    c.width = 1280;
    c.height = 720;
    c.fx = 1200.0;
    c.fy = 1200.0;
    c.cx = 640.0;
    c.cy = 360.0;
    c.k1 = -0.12;
    c.k2 = 0.05;
    c.p1 = 0.0005;
    c.p2 = -0.0003;
    c.k3 = 0.0;
    return c;
}

GimbalConfig defaultGimbal() {
    GimbalConfig g;
    // 相机 x 向右 = 云台 -y；相机 y 向下 = 云台 -z；相机 z 向前 = 云台 +x
    g.R_cam_to_gimbal = Mat3::fromColumns({0, -1, 0}, {0, 0, -1}, {1, 0, 0});
    g.t_cam_in_gimbal = {0.05, 0.0, 0.03};
    g.t_muzzle_in_gimbal = {0.12, 0.0, -0.04};
    g.yaw_min = -80.0 * kDeg2Rad;
    g.yaw_max = 80.0 * kDeg2Rad;
    g.pitch_min = -20.0 * kDeg2Rad;
    g.pitch_max = 30.0 * kDeg2Rad;
    return g;
}

struct Scene {
    Mat3 R;
    Vec3 t;
    std::array<Vec2, 4> uv{};
    double yawCurrent{0.0};
    double pitchCurrent{0.0};
    AimSolution truth;
};

Mat3 targetRotation(const Vec3& direction, std::mt19937& rng, double tiltDeg, bool roll) {
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    const Vec3 r3 = (-direction).normalized();
    Vec3 r1 = cross(Vec3{0.0, -1.0, 0.0}, r3);
    if (r1.norm() < 1e-9) r1 = Vec3{1.0, 0.0, 0.0};
    r1 = r1.normalized();
    const Vec3 r2 = cross(r3, r1);
    Mat3 R = Mat3::fromColumns(r1, r2, r3);
    R = R * rotX(uni(rng) * tiltDeg * kDeg2Rad) * rotY(uni(rng) * tiltDeg * kDeg2Rad);
    if (roll) R = R * rotZ(uni(rng) * kPi);
    return R;
}

bool makeScene(std::mt19937& rng, const Intrinsics& cam, const TargetModel& target,
               const AimSolver& solver, double dMin, double dMax, bool allowRoll, double marginPx,
               Scene& out) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const double hfov = cam.hfovDeg() * kDeg2Rad * 0.5;
    const double vfov = cam.vfovDeg() * kDeg2Rad * 0.5;
    const auto corners3d = target.points();

    for (int attempt = 0; attempt < 200; ++attempt) {
        const double az = (uni(rng) * 2.0 - 1.0) * 0.65 * hfov;
        const double el = (uni(rng) * 2.0 - 1.0) * 0.65 * vfov;
        const double d = dMin + (dMax - dMin) * uni(rng);
        const Vec3 dir{std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el)};

        out.t = dir * d;
        out.R = targetRotation(dir, rng, 25.0, allowRoll);

        bool visible = true;
        for (int i = 0; i < 4; ++i) {
            const Vec3 p = out.R * corners3d[i] + out.t;
            if (p.z <= 0.2) {
                visible = false;
                break;
            }
            const Vec2 uv = cam.project(p);
            if (!std::isfinite(uv.x) || !std::isfinite(uv.y) || uv.x < marginPx ||
                uv.x > cam.width - marginPx || uv.y < marginPx || uv.y > cam.height - marginPx) {
                visible = false;
                break;
            }
            out.uv[i] = uv;
        }
        if (!visible) continue;

        out.yawCurrent = (uni(rng) * 2.0 - 1.0) * 0.5;
        out.pitchCurrent = -0.2 + 0.5 * uni(rng);
        out.truth = solver.solve(out.t, out.yawCurrent, out.pitchCurrent);
        return true;
    }
    return false;
}

struct Row {
    double sigma{0.0};
    double distance{0.0};
    double yawErr{0.0};
    double pitchErr{0.0};
    double tErrMm{0.0};
    double rotErr{0.0};
    double reproj{0.0};
    double baseYawErr{0.0};
    double basePitchErr{0.0};
    bool accepted{false};
};

struct Stats {
    double mean{0.0};
    double rms{0.0};
    double p95{0.0};
    double max{0.0};
    size_t n{0};
};

Stats computeStats(const std::vector<double>& v) {
    Stats s;
    if (v.empty()) return s;
    s.n = v.size();
    double sum = 0.0, sq = 0.0;
    for (double x : v) {
        sum += x;
        sq += x * x;
    }
    s.mean = sum / static_cast<double>(v.size());
    s.rms = std::sqrt(sq / static_cast<double>(v.size()));
    std::vector<double> sorted = v;
    std::sort(sorted.begin(), sorted.end());
    const double idx = 0.95 * static_cast<double>(sorted.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = std::min(lo + 1, sorted.size() - 1);
    const double frac = idx - static_cast<double>(lo);
    s.p95 = sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    s.max = sorted.back();
    return s;
}

std::vector<Row> runTrials(const Intrinsics& cam, const TargetModel& target,
                           const GimbalConfig& gimbal, int trials, double sigma, unsigned seed,
                           double dMin, double dMax) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma);
    AimSolver solver(gimbal);
    PoseEstimator estimator(cam, target, gimbal);

    std::vector<Row> rows;
    rows.reserve(static_cast<size_t>(trials));
    int guard = 0;
    while (static_cast<int>(rows.size()) < trials && guard < trials * 20) {
        ++guard;
        Scene scene;
        if (!makeScene(rng, cam, target, solver, dMin, dMax, true, 10.0, scene)) continue;

        std::array<Vec2, 4> uv;
        for (int i = 0; i < 4; ++i) uv[i] = {scene.uv[i].x + noise(rng), scene.uv[i].y + noise(rng)};

        const AimEstimate est = estimator.aim(uv, scene.yawCurrent, scene.pitchCurrent);

        Vec2 center;
        for (const auto& p : uv) center += p;
        center = center * 0.25;
        // 朴素基线：像素偏移直接当角度增量。注意符号——相机 x 向右，而云台 y 向左，
        // 因此“目标在画面右侧”对应 yaw 减小；pitch 同理（相机 y 向下）。
        const double baseYaw = scene.yawCurrent - std::atan2(center.x - cam.cx, cam.fx);
        const double basePitch = scene.pitchCurrent - std::atan2(center.y - cam.cy, cam.fy);

        Row r;
        r.sigma = sigma;
        r.distance = scene.t.norm();
        r.yawErr = std::fabs(wrapAngle(est.aim.yaw - scene.truth.yaw)) * kRad2Deg;
        r.pitchErr = std::fabs(est.aim.pitch - scene.truth.pitch) * kRad2Deg;
        r.tErrMm = (est.pose.t - scene.t).norm() * 1000.0;
        r.rotErr = rotationAngleDeg(est.pose.R, scene.R);
        r.reproj = est.pose.reprojRmsPx;
        r.baseYawErr = std::fabs(wrapAngle(baseYaw - scene.truth.yaw)) * kRad2Deg;
        r.basePitchErr = std::fabs(basePitch - scene.truth.pitch) * kRad2Deg;
        r.accepted = est.accepted;
        rows.push_back(r);
    }
    return rows;
}

void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "sigma_px,distance_m,yaw_err_deg,pitch_err_deg,t_err_mm,rot_err_deg,reproj_rms_px,"
         "baseline_yaw_err_deg,baseline_pitch_err_deg,accepted\n";
    f.setf(std::ios::fixed);
    f.precision(6);
    for (const auto& r : rows) {
        f << r.sigma << ',' << r.distance << ',' << r.yawErr << ',' << r.pitchErr << ','
          << r.tErrMm << ',' << r.rotErr << ',' << r.reproj << ',' << r.baseYawErr << ','
          << r.basePitchErr << ',' << (r.accepted ? 1 : 0) << '\n';
    }
}

struct Summary {
    double key{0.0};
    size_t n{0};
    Stats yaw, pitch, t, rot, reproj, baseYaw, basePitch;
    double acceptRate{0.0};
};

Summary summarize(const std::vector<Row>& rows) {
    Summary s;
    s.n = rows.size();
    std::vector<double> yaw, pitch, t, rot, reproj, baseYaw, basePitch;
    double accepted = 0.0;
    for (const auto& r : rows) {
        yaw.push_back(r.yawErr);
        pitch.push_back(r.pitchErr);
        t.push_back(r.tErrMm);
        rot.push_back(r.rotErr);
        reproj.push_back(r.reproj);
        baseYaw.push_back(r.baseYawErr);
        basePitch.push_back(r.basePitchErr);
        accepted += r.accepted ? 1.0 : 0.0;
    }
    s.yaw = computeStats(yaw);
    s.pitch = computeStats(pitch);
    s.t = computeStats(t);
    s.rot = computeStats(rot);
    s.reproj = computeStats(reproj);
    s.baseYaw = computeStats(baseYaw);
    s.basePitch = computeStats(basePitch);
    s.acceptRate = s.n ? accepted / static_cast<double>(s.n) : 0.0;
    return s;
}

void writeReport(const std::string& path, const std::vector<Summary>& noise,
                 const std::vector<Summary>& dist, const Intrinsics& cam, const TargetModel& target,
                 const GimbalConfig& gimbal, int trials) {
    std::ofstream f(path);
    f << "# 靶位姿解算与瞄准转角 合成数据验证\n\n";
    f << "- 程序：`algorithm/cpp/apps/pose_selftest.cpp`\n";
    f << "- 每组试验次数：" << trials << "\n\n";

    f << "## 1 配置\n\n| 项目 | 值 |\n| --- | --- |\n";
    f << "| 分辨率 | " << cam.width << "×" << cam.height << " |\n";
    f << "| 内参 | fx=" << cam.fx << ", fy=" << cam.fy << ", cx=" << cam.cx << ", cy=" << cam.cy << " |\n";
    f << "| 畸变 | k1=" << cam.k1 << ", k2=" << cam.k2 << ", p1=" << cam.p1 << ", p2=" << cam.p2
      << ", k3=" << cam.k3 << " |\n";
    f << "| 靶标边长 | " << target.size * 1000.0 << " mm |\n";
    f << "| 相机在云台系的位置 | " << gimbal.t_cam_in_gimbal.x << ", " << gimbal.t_cam_in_gimbal.y
      << ", " << gimbal.t_cam_in_gimbal.z << " m |\n";
    f << "| 枪口在云台系的位置 | " << gimbal.t_muzzle_in_gimbal.x << ", "
      << gimbal.t_muzzle_in_gimbal.y << ", " << gimbal.t_muzzle_in_gimbal.z << " m |\n";
    f << "| 靶面倾角 | ±25° |\n| 靶距范围 | 2–8 m |\n\n";

    f << "## 2 像素噪声 -> 角度误差\n\n";
    f << "| 角点噪声 σ (px) | 试验数 | yaw RMS (°) | yaw P95 (°) | pitch RMS (°) | pitch P95 (°) | "
         "位姿平移 RMS (mm) | 姿态 RMS (°) | 重投影 RMS (px) | 接受率 |\n";
    f << "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n";
    f.setf(std::ios::fixed);
    for (const auto& s : noise) {
        f.precision(3);
        f << "| " << s.key << " | " << s.n << " | " << s.yaw.rms << " | " << s.yaw.p95 << " | "
          << s.pitch.rms << " | " << s.pitch.p95 << " | ";
        f.precision(2);
        f << s.t.rms << " | ";
        f.precision(3);
        f << s.rot.rms << " | " << s.reproj.mean << " | ";
        f.precision(1);
        f << s.acceptRate * 100.0 << "% |\n";
    }

    f << "\n## 3 与“像素偏移当角度”的朴素做法对比\n\n";
    f << "| 角点噪声 σ (px) | 本文 yaw RMS (°) | 朴素 yaw RMS (°) | 本文 pitch RMS (°) | 朴素 pitch RMS (°) |\n";
    f << "| --- | --- | --- | --- | --- |\n";
    for (const auto& s : noise) {
        f.precision(3);
        f << "| " << s.key << " | " << s.yaw.rms << " | " << s.baseYaw.rms << " | " << s.pitch.rms
          << " | " << s.basePitch.rms << " |\n";
    }

    f << "\n## 4 距离扫描（σ = 1 px）\n\n";
    f << "| 靶距 (m) | 试验数 | yaw RMS (°) | pitch RMS (°) | yaw P95 (°) | 位姿平移 RMS (mm) | "
         "落点偏差 RMS (cm) |\n";
    f << "| --- | --- | --- | --- | --- | --- | --- |\n";
    for (const auto& s : dist) {
        const double impactCm = std::tan(s.yaw.rms * kDeg2Rad) * s.key * 100.0;
        f.precision(1);
        f << "| " << s.key << " | " << s.n << " | ";
        f.precision(3);
        f << s.yaw.rms << " | " << s.pitch.rms << " | " << s.yaw.p95 << " | ";
        f.precision(2);
        f << s.t.rms << " | ";
        f.precision(2);
        f << impactCm << " |\n";
    }
    f << "\n真值由无噪声几何直接正投影得到，不依赖 PnP；误差统计为“估计 − 真值”。\n";
}

}  // namespace

int main(int argc, char** argv) {
    const std::string outDir = argc > 1 ? argv[1] : "out";
    const int trials = argc > 2 ? std::atoi(argv[2]) : 300;

    const Intrinsics cam = defaultCamera();
    const TargetModel target;
    const GimbalConfig gimbal = defaultGimbal();

    std::printf("camera %dx%d fx=%.1f fy=%.1f, target %.0f mm, trials=%d\n", cam.width, cam.height,
                cam.fx, cam.fy, target.size * 1000.0, trials);

    // 1) 噪声扫描
    std::vector<Summary> noiseSummaries;
    std::vector<Row> allRows;
    const double sigmas[] = {0.0, 0.5, 1.0, 2.0};
    for (int k = 0; k < 4; ++k) {
        const std::vector<Row> rows =
            runTrials(cam, target, gimbal, trials, sigmas[k], 12345u + 7u * static_cast<unsigned>(k),
                      2.0, 8.0);
        Summary s = summarize(rows);
        s.key = sigmas[k];
        noiseSummaries.push_back(s);
        allRows.insert(allRows.end(), rows.begin(), rows.end());
        std::printf("sigma=%.2fpx  yaw RMS=%.3f deg  pitch RMS=%.3f deg  t RMS=%.2f mm  "
                    "baseline yaw RMS=%.3f deg\n",
                    sigmas[k], s.yaw.rms, s.pitch.rms, s.t.rms, s.baseYaw.rms);
    }

    // 2) 距离扫描（σ = 1 px）
    std::vector<Summary> distSummaries;
    const double dists[] = {2.0, 4.0, 6.0, 8.0};
    for (int k = 0; k < 4; ++k) {
        const std::vector<Row> rows = runTrials(cam, target, gimbal, std::max(20, trials / 2), 1.0,
                                                777u + 13u * static_cast<unsigned>(k), dists[k],
                                                dists[k] + 0.2);
        Summary s = summarize(rows);
        s.key = dists[k];
        distSummaries.push_back(s);
        writeCsv(outDir + "/raw_distance_" + std::to_string(static_cast<int>(dists[k])) + "m.csv", rows);
        std::printf("dist=%.0fm  yaw RMS=%.3f deg  pitch RMS=%.3f deg  t RMS=%.2f mm\n", dists[k],
                    s.yaw.rms, s.pitch.rms, s.t.rms);
    }

    // 3) 闭环自检：按解算角转过去，枪管应对准靶心
    {
        std::mt19937 rng(99);
        AimSolver solver(gimbal);
        PoseEstimator estimator(cam, target, gimbal);
        double worst = 0.0;
        double sumIters = 0.0;
        int count = 0;
        for (int i = 0; i < std::max(50, trials / 2); ++i) {
            Scene scene;
            if (!makeScene(rng, cam, target, solver, 2.0, 8.0, true, 10.0, scene)) continue;
            const AimEstimate est = estimator.aim(scene.uv, scene.yawCurrent, scene.pitchCurrent);
            const Vec2 err = solver.aimError(est.pose.t, scene.yawCurrent, scene.pitchCurrent,
                                             est.aim.yaw, est.aim.pitch);
            worst = std::max(worst, err.norm());
            sumIters += est.aim.iterations;
            ++count;
        }
        std::printf("closure check: max residual = %.3e deg (%.3e rad), mean iters = %.2f\n",
                    worst * kRad2Deg, worst, sumIters / std::max(1, count));
    }

    writeCsv(outDir + "/raw_noise.csv", allRows);
    writeReport(outDir + "/report.md", noiseSummaries, distSummaries, cam, target, gimbal, trials);
    std::printf("report written to %s/report.md\n", outDir.c_str());
    return 0;
}
