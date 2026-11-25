#include "bff/rune_detector.hpp"

#include <algorithm>
#include <cmath>

namespace bff {

const char* runeStateName(RuneState state) {
    switch (state) {
        case RuneState::Inactive:
            return "INACTIVE";
        case RuneState::Active:
            return "ACTIVE";
        default:
            return "UNKNOWN";
    }
}

void RuneDetector::reset() {
    window_.clear();
    committed_ = RuneState::Unknown;
    candidate_ = RuneState::Unknown;
    candidateCount_ = 0;
}

RuneObservation RuneDetector::process(const Image& bgr, double timestampSec) {
    RuneObservation out;
    if (bgr.empty()) return out;

    // 降采样：实时场景下把处理量降到 1/4，坐标最后再乘回去
    const int ds = std::max(1, cfg_.downsample);
    Image scaled;
    const Image* work = &bgr;
    if (ds > 1) {
        scaled = downscale(bgr, ds);
        if (!scaled.empty()) work = &scaled;
    }
    const double inv = 1.0 / static_cast<double>(ds);

    const HsvPlanes hsv = bgrToHsv(*work);

    BlobParams params = cfg_.blob;
    params.minArea = std::max(6, static_cast<int>(static_cast<double>(cfg_.blob.minArea) * inv * inv));
    params.maxArea =
        std::max(params.minArea + 1,
                 static_cast<int>(static_cast<double>(cfg_.blob.maxArea) * inv * inv));

    const std::vector<Blob> blobs = findLedBlobs(*work, hsv, cfg_.vMin, cfg_.sMin, params);
    if (blobs.empty()) {
        candidateCount_ = 0;
        candidate_ = RuneState::Unknown;
        committed_ = RuneState::Unknown;
        window_.clear();
        return out;
    }

    const double clusterRadius = static_cast<double>(cfg_.clusterRadiusPx) * inv;

    // 以最大亮点为种子聚类，再用组内质心重聚一次，避免只框住单个叶片
    std::vector<int> group;
    auto collect = [&](const Vec2& seed) {
        group.clear();
        for (int i = 0; i < static_cast<int>(blobs.size()); ++i) {
            if ((blobs[i].centroid - seed).norm() <= clusterRadius) group.push_back(i);
        }
    };
    collect(blobs.front().centroid);
    Vec2 center;
    double totalArea = 0.0;
    for (int idx : group) {
        center += blobs[idx].centroid * static_cast<double>(blobs[idx].area);
        totalArea += blobs[idx].area;
    }
    if (totalArea > 0.0) {
        center = center * (1.0 / totalArea);
        collect(center);
    }
    if (group.empty()) return out;

    double sumV = 0.0, sumS = 0.0, sumCos = 0.0, sumSin = 0.0, hueActiveArea = 0.0, area = 0.0;
    Rect box;
    std::vector<Rect> boxes;
    for (int idx : group) {
        const Blob& b = blobs[idx];
        box = Rect::united(box, b.box);
        boxes.push_back(b.box);
        sumV += b.meanV * b.area;
        sumS += b.meanS * b.area;
        const double hueRad = b.meanH * (2.0 * kPi / 180.0);
        sumCos += std::cos(hueRad) * b.area;
        sumSin += std::sin(hueRad) * b.area;
        area += b.area;
        if (hueDistance(static_cast<int>(std::lround(b.meanH)), cfg_.hueActiveCenter) <=
            cfg_.hueActiveTolerance) {
            hueActiveArea += b.area;
        }
    }
    if (area <= 0.0) return out;

    RuneFeatures f;
    f.meanV = sumV / area;
    f.meanS = sumS / area;
    double hueDeg = std::atan2(sumSin, sumCos) * kRad2Deg;
    if (hueDeg < 0.0) hueDeg += 360.0;
    f.meanH = hueDeg * 0.5;
    f.activeHueRatio = hueActiveArea / area;
    f.blobCount = static_cast<int>(group.size());

    // 坐标还原到原图
    const int margin = static_cast<int>(std::lround(cfg_.boxMarginPx * inv));
    const Rect boxWork = box.expanded(margin);
    out.box = {boxWork.x * ds, boxWork.y * ds, boxWork.w * ds, boxWork.h * ds};
    out.blobBoxes.reserve(boxes.size());
    for (const auto& b : boxes) out.blobBoxes.push_back({b.x * ds, b.y * ds, b.w * ds, b.h * ds});

    out.detected = true;

    window_.push_back({timestampSec, f.meanV, f.activeHueRatio, f.blobCount});
    while (!window_.empty() && (timestampSec - window_.front().t > cfg_.windowSeconds ||
                                static_cast<int>(window_.size()) > cfg_.windowFrames)) {
        window_.pop_front();
    }
    temporalFeatures(f, timestampSec);
    out.features = f;

    const double sHue = f.activeHueRatio >= cfg_.activeHueRatioOn ? 1.0 : 0.0;
    const double sMod = f.modulationDepth >= cfg_.modulationOn ? 1.0 : 0.0;
    const double sBlink = (f.modulationDepth >= 0.1 && f.blinkHz >= cfg_.blinkHzMin &&
                           f.blinkHz <= cfg_.blinkHzMax)
                              ? 1.0
                              : 0.0;
    const double sBright = f.meanV >= cfg_.brightnessActive ? 1.0 : 0.0;

    out.scoreActive = cfg_.wHue * sHue + cfg_.wModulation * sMod + cfg_.wBlink * sBlink +
                      cfg_.wBrightness * sBright;
    out.candidate = out.scoreActive >= cfg_.scoreThreshold ? RuneState::Active : RuneState::Inactive;
    out.confidence = std::fabs(2.0 * out.scoreActive - 1.0);

    if (out.candidate != candidate_) {
        candidate_ = out.candidate;
        candidateCount_ = 1;
    } else {
        ++candidateCount_;
    }
    if (candidateCount_ >= cfg_.switchFrames) committed_ = candidate_;

    out.state = committed_;
    out.stabilized = committed_ != RuneState::Unknown && committed_ == out.candidate;
    return out;
}

void RuneDetector::temporalFeatures(RuneFeatures& f, double now) const {
    if (window_.size() < 3) {
        f.modulationDepth = 0.0;
        f.blinkHz = 0.0;
        f.dutyCycle = 1.0;
        return;
    }
    double mn = window_.front().meanV;
    double mx = window_.front().meanV;
    double sum = 0.0;
    for (const auto& s : window_) {
        mn = std::min(mn, s.meanV);
        mx = std::max(mx, s.meanV);
        sum += s.meanV;
    }
    const double mean = sum / static_cast<double>(window_.size());
    f.modulationDepth = (mx - mn) / std::max(mean, 1.0);

    int above = 0;
    for (const auto& s : window_) above += s.meanV > mean ? 1 : 0;
    f.dutyCycle = static_cast<double>(above) / static_cast<double>(window_.size());

    int crossings = 0;
    double prev = window_.front().meanV - mean;
    for (size_t i = 1; i < window_.size(); ++i) {
        const double cur = window_[i].meanV - mean;
        if ((prev > 0.0 && cur <= 0.0) || (prev < 0.0 && cur >= 0.0)) ++crossings;
        prev = cur;
    }
    const double duration = std::max(now - window_.front().t, 1e-3);
    f.blinkHz = 0.5 * static_cast<double>(crossings) / duration;
    if (window_.size() < 8) f.blinkHz = 0.0;  // 窗口太短，频率不可信
}

}  // namespace bff
