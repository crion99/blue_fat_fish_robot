// 能量机关灯效识别：“待激活（INACTIVE）” / “已激活（ACTIVE）” + 图像标注框。
//
// 判别思路（两级）：
//   空间：亮度+饱和度阈值取 LED 亮点 -> 连通域 -> 以最大亮点为种子聚类，得到灯效区域；
//   时域：在 1.5 s 滑窗内统计亮度调制深度、闪烁频率、占空比、激活色相占比，
//         加权打分后做迟滞判决（连续 N 帧一致才切换），避免单帧误判。
//
// 阈值全部集中在 RuneConfig，对应 configs/rune_states.yml；
// 出厂默认值按“待激活=蓝色常亮、已激活=红色高亮/闪烁”设定，实机需用标定程序重新拟合。

#pragma once

#include <deque>
#include <string>
#include <vector>

#include "bff/blobs.hpp"
#include "bff/color.hpp"
#include "bff/image.hpp"

namespace bff {

enum class RuneState { Unknown = 0, Inactive = 1, Active = 2 };

const char* runeStateName(RuneState state);

struct RuneFeatures {
    double meanV{0.0};
    double meanS{0.0};
    double meanH{0.0};
    double activeHueRatio{0.0};
    int blobCount{0};
    double modulationDepth{0.0};
    double blinkHz{0.0};
    double dutyCycle{0.0};
};

struct RuneObservation {
    RuneState state{RuneState::Unknown};      // 迟滞后的稳定输出
    RuneState candidate{RuneState::Unknown};  // 当前单帧判决
    bool stabilized{false};
    bool detected{false};
    double confidence{0.0};
    double scoreActive{0.0};
    Rect box;
    std::vector<Rect> blobBoxes;
    RuneFeatures features;
};

struct RuneConfig {
    // 空间检测
    int vMin{110};
    int sMin{85};
    int downsample{2};  // 1=原始分辨率；2=先 2×2 均值降采样，处理量约为 1/4
    BlobParams blob{};
    int clusterRadiusPx{320};
    int boxMarginPx{16};

    // 颜色判据（H ∈ 0..179）
    int hueActiveCenter{5};
    int hueActiveTolerance{14};
    int hueInactiveCenter{105};
    int hueInactiveTolerance{25};
    double activeHueRatioOn{0.50};

    // 时域判据
    double brightnessActive{190.0};
    double modulationOn{0.20};
    double blinkHzMin{0.5};
    double blinkHzMax{15.0};
    int windowFrames{45};
    double windowSeconds{1.5};
    int switchFrames{5};
    double scoreThreshold{0.50};

    // 打分权重（和为 1）
    double wHue{0.45};
    double wModulation{0.25};
    double wBlink{0.15};
    double wBrightness{0.15};
};

class RuneDetector {
public:
    explicit RuneDetector(const RuneConfig& cfg = RuneConfig{}) : cfg_(cfg) {}

    void reset();

    // timestampSec 由采集端提供（秒，单调递增）；用于计算窗口时长与闪烁频率
    RuneObservation process(const Image& bgr, double timestampSec);

    const RuneConfig& config() const { return cfg_; }
    void setConfig(const RuneConfig& cfg) { cfg_ = cfg; }

private:
    struct Sample {
        double t{0.0};
        double meanV{0.0};
        double activeHueRatio{0.0};
        int blobCount{0};
    };

    void temporalFeatures(RuneFeatures& f, double now) const;

    RuneConfig cfg_;
    std::deque<Sample> window_;
    RuneState committed_{RuneState::Unknown};
    RuneState candidate_{RuneState::Unknown};
    int candidateCount_{0};
};

}  // namespace bff
