// 合成能量机关画面：用于在没有相机/场地时验证灯效识别流程与时域逻辑。
//
// 只是“像”而不是物理仿真：用图像空间近似投影（按距离给缩放），
// 叶片、LED、中心靶面都是简单图元 + 均匀噪声。
// 真实判别阈值必须用实拍数据重新标定（见 README 的标定步骤）。

#pragma once

#include <cstdint>
#include <random>

#include "bff/image.hpp"

namespace bff {

struct RuneSceneConfig {
    int width{1280};
    int height{720};
    double centerX{640.0};
    double centerY{360.0};
    double ringRadiusPx{110.0};
    double scale{1.0};
    double rotationDeg{0.0};

    bool active{true};
    double brightness{235.0};      // 已激活 LED 亮度（0..255）
    double dimBrightness{140.0};   // 待激活 LED 亮度
    double flickerHz{4.0};
    double dutyCycle{0.5};
    int hueActive{5};      // 偏红
    int hueInactive{105};  // 偏蓝
    int saturation{255};

    double noiseAmplitude{6.0};
    double background{22.0};
    double bladeGray{70.0};
    double targetSquarePx{46.0};
};

Image renderRuneFrame(const RuneSceneConfig& cfg, double tSeconds, std::mt19937& rng);

// 背景帧（无能量机关），用于统计误检
Image renderEmptyFrame(const RuneSceneConfig& cfg, std::mt19937& rng);

}  // namespace bff
