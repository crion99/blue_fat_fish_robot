#ifdef BFF_WITH_OPENCV

#include "bff/config.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace bff {
namespace {

bool openFile(const std::string& path, cv::FileStorage& fs, std::string& error) {
    try {
        fs.open(path, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        error = e.what();
        return false;
    }
    if (!fs.isOpened() || fs.root().empty()) {
        error = "打不开配置文件: " + path + "（检查路径与 YAML 格式）";
        return false;
    }
    return true;
}

double readDouble(const cv::FileNode& node, const char* key, double fallback) {
    const cv::FileNode n = node[key];
    if (n.empty()) return fallback;
    double v = fallback;
    n >> v;
    return v;
}

int readInt(const cv::FileNode& node, const char* key, int fallback) {
    const cv::FileNode n = node[key];
    if (n.empty()) return fallback;
    int v = fallback;
    n >> v;
    return v;
}

std::string readString(const cv::FileNode& node, const char* key, const std::string& fallback) {
    const cv::FileNode n = node[key];
    if (n.empty()) return fallback;
    std::string v;
    n >> v;
    return v.empty() ? fallback : v;
}

bool readVec3(const cv::FileNode& node, const char* key, Vec3& out) {
    const cv::FileNode n = node[key];
    if (n.empty() || !n.isSeq() || n.size() != 3) return false;
    out = {static_cast<double>(n[0]), static_cast<double>(n[1]), static_cast<double>(n[2])};
    return true;
}

}  // namespace

bool loadCameraConfig(const std::string& path, Intrinsics& out, std::string& error) {
    cv::FileStorage fs;
    if (!openFile(path, fs, error)) return false;
    const cv::FileNode n = fs.root();

    out.name = readString(n, "name", "camera");
    out.fx = readDouble(n, "fx", out.fx);
    out.fy = readDouble(n, "fy", out.fy);
    out.cx = readDouble(n, "cx", out.cx);
    out.cy = readDouble(n, "cy", out.cy);
    out.width = readInt(n, "width", out.width);
    out.height = readInt(n, "height", out.height);

    const cv::FileNode dist = n["dist"];
    if (!dist.empty() && dist.isSeq() && dist.size() >= 5) {
        out.k1 = static_cast<double>(dist[0]);
        out.k2 = static_cast<double>(dist[1]);
        out.p1 = static_cast<double>(dist[2]);
        out.p2 = static_cast<double>(dist[3]);
        out.k3 = static_cast<double>(dist[4]);
    }
    return true;
}

bool loadGimbalConfig(const std::string& path, GimbalConfig& out, std::string& error) {
    cv::FileStorage fs;
    if (!openFile(path, fs, error)) return false;
    const cv::FileNode n = fs.root();

    const cv::FileNode rot = n["R_cam_to_gimbal"];
    if (!rot.empty() && rot.isSeq() && rot.size() == 9) {
        double v[9];
        for (int i = 0; i < 9; ++i) v[i] = static_cast<double>(rot[i]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) out.R_cam_to_gimbal.m[r][c] = v[3 * r + c];
    }
    readVec3(n, "t_cam_in_gimbal", out.t_cam_in_gimbal);
    readVec3(n, "t_muzzle_in_gimbal", out.t_muzzle_in_gimbal);

    out.yaw_min = readDouble(n, "yaw_min_deg", out.yaw_min * kRad2Deg) * kDeg2Rad;
    out.yaw_max = readDouble(n, "yaw_max_deg", out.yaw_max * kRad2Deg) * kDeg2Rad;
    out.pitch_min = readDouble(n, "pitch_min_deg", out.pitch_min * kRad2Deg) * kDeg2Rad;
    out.pitch_max = readDouble(n, "pitch_max_deg", out.pitch_max * kRad2Deg) * kDeg2Rad;
    out.gravity = readDouble(n, "gravity", out.gravity);
    out.muzzle_speed = readDouble(n, "muzzle_speed", out.muzzle_speed);
    return true;
}

bool loadTargetConfig(const std::string& path, TargetModel& out, std::string& error) {
    cv::FileStorage fs;
    if (!openFile(path, fs, error)) return false;
    const cv::FileNode n = fs.root();
    out.size = readDouble(n, "size", out.size);
    out.name = readString(n, "name", out.name);
    if (out.size <= 0.0) {
        error = "靶标边长必须为正（单位：米）";
        return false;
    }
    return true;
}

bool loadRuneConfig(const std::string& path, RuneConfig& out, std::string& error) {
    cv::FileStorage fs;
    if (!openFile(path, fs, error)) return false;
    const cv::FileNode n = fs.root();

    out.vMin = readInt(n, "vMin", out.vMin);
    out.sMin = readInt(n, "sMin", out.sMin);
    out.downsample = readInt(n, "downsample", out.downsample);
    out.clusterRadiusPx = readInt(n, "clusterRadiusPx", out.clusterRadiusPx);
    out.boxMarginPx = readInt(n, "boxMarginPx", out.boxMarginPx);
    out.blob.minArea = readInt(n, "blobMinArea", out.blob.minArea);
    out.blob.maxArea = readInt(n, "blobMaxArea", out.blob.maxArea);
    out.blob.maxAspect = readDouble(n, "blobMaxAspect", out.blob.maxAspect);

    out.hueActiveCenter = readInt(n, "hueActiveCenter", out.hueActiveCenter);
    out.hueActiveTolerance = readInt(n, "hueActiveTolerance", out.hueActiveTolerance);
    out.hueInactiveCenter = readInt(n, "hueInactiveCenter", out.hueInactiveCenter);
    out.hueInactiveTolerance = readInt(n, "hueInactiveTolerance", out.hueInactiveTolerance);
    out.activeHueRatioOn = readDouble(n, "activeHueRatioOn", out.activeHueRatioOn);

    out.brightnessActive = readDouble(n, "brightnessActive", out.brightnessActive);
    out.modulationOn = readDouble(n, "modulationOn", out.modulationOn);
    out.blinkHzMin = readDouble(n, "blinkHzMin", out.blinkHzMin);
    out.blinkHzMax = readDouble(n, "blinkHzMax", out.blinkHzMax);
    out.windowFrames = readInt(n, "windowFrames", out.windowFrames);
    out.windowSeconds = readDouble(n, "windowSeconds", out.windowSeconds);
    out.switchFrames = readInt(n, "switchFrames", out.switchFrames);
    out.scoreThreshold = readDouble(n, "scoreThreshold", out.scoreThreshold);
    return true;
}

}  // namespace bff

#endif  // BFF_WITH_OPENCV
