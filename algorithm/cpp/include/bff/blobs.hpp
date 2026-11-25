// LED 亮点连通域提取（BFS 标记 + 形状/颜色统计）。

#pragma once

#include <vector>

#include "bff/color.hpp"
#include "bff/image.hpp"
#include "bff/linalg.hpp"

namespace bff {

struct Blob {
    Rect box;
    int area{0};
    Vec2 centroid;
    double meanH{0.0};  // 色相圆周平均
    double meanS{0.0};
    double meanV{0.0};
};

struct BlobParams {
    int minArea{40};        // 小于该像素数视为噪声
    int maxArea{120000};    // 大于该面积视为大面积反光/背景
    double maxAspect{8.0};  // 长宽比上限
    int hueTolerance{12};   // 参与圆周平均的色相窗口（仅用于统计，不做筛选）
};

// vMin 为亮度阈值（0..255），sMin 为饱和度阈值（0..255）
std::vector<Blob> findLedBlobs(const Image& bgr, const HsvPlanes& hsv, int vMin, int sMin,
                               const BlobParams& params);

}  // namespace bff
