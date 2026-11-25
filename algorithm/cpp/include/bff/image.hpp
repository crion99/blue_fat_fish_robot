// 最小图像容器（BGR888，行优先），不依赖 OpenCV，便于在无相机环境下跑单元测试。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace bff {

struct Rect {
    int x{0};
    int y{0};
    int w{0};
    int h{0};

    bool empty() const { return w <= 0 || h <= 0; }
    int area() const { return empty() ? 0 : w * h; }
    int right() const { return x + w; }
    int bottom() const { return y + h; }

    Rect expanded(int m) const { return {x - m, y - m, w + 2 * m, h + 2 * m}; }

    bool contains(int px, int py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }

    static Rect united(const Rect& a, const Rect& b) {
        if (a.empty()) return b;
        if (b.empty()) return a;
        const int x0 = a.x < b.x ? a.x : b.x;
        const int y0 = a.y < b.y ? a.y : b.y;
        const int x1 = a.right() > b.right() ? a.right() : b.right();
        const int y1 = a.bottom() > b.bottom() ? a.bottom() : b.bottom();
        return {x0, y0, x1 - x0, y1 - y0};
    }
};

struct Image {
    int width{0};
    int height{0};
    std::vector<uint8_t> data;  // BGR，size = w*h*3

    Image() = default;
    Image(int w, int h) : width(w), height(h), data(static_cast<size_t>(w) * h * 3, 0) {}

    bool empty() const {
        return width <= 0 || height <= 0 || data.size() != static_cast<size_t>(width) * height * 3;
    }

    uint8_t* row(int y) { return data.data() + static_cast<size_t>(y) * width * 3; }
    const uint8_t* row(int y) const { return data.data() + static_cast<size_t>(y) * width * 3; }

    uint8_t* pixel(int x, int y) {
        return data.data() + (static_cast<size_t>(y) * width + x) * 3;
    }
    const uint8_t* pixel(int x, int y) const {
        return data.data() + (static_cast<size_t>(y) * width + x) * 3;
    }

    // 写 PPM(P6)：不需要任何第三方库就能查看结果，方便在没有 OpenCV 的机器上留证据
    bool savePpm(const std::string& path) const;
};

// 2×2（或更大）均值降采样，用于实时场景降低处理量
Image downscale(const Image& src, int factor);

}  // namespace bff
