// HSV 转换与 LED 亮度/色相统计（与 OpenCV 的 8 位 HSV 约定一致：
// H ∈ [0,180)，S ∈ [0,255]，V ∈ [0,255]，方便和 cv::cvtColor 的结果对照）。

#pragma once

#include <cstdint>
#include <vector>

#include "bff/image.hpp"

namespace bff {

struct HsvPlanes {
    std::vector<uint8_t> h;
    std::vector<uint8_t> s;
    std::vector<uint8_t> v;
    int width{0};
    int height{0};

    uint8_t hue(int x, int y) const { return h[static_cast<size_t>(y) * width + x]; }
    uint8_t sat(int x, int y) const { return s[static_cast<size_t>(y) * width + x]; }
    uint8_t val(int x, int y) const { return v[static_cast<size_t>(y) * width + x]; }
};

HsvPlanes bgrToHsv(const Image& bgr);

// 色相圆环距离（H 取值 0..179，环绕）
int hueDistance(int a, int b);

}  // namespace bff
