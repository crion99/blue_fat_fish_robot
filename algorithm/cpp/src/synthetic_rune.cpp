#include "bff/synthetic_rune.hpp"

#include <algorithm>
#include <cmath>

#include "bff/linalg.hpp"

namespace bff {
namespace {

struct Bgr {
    uint8_t b, g, r;
};

Bgr hsvToBgr(int h, int s, int v) {
    // h ∈ [0,180)，s、v ∈ [0,255]，与 bff::bgrToHsv 互逆
    const double hh = std::fmod(static_cast<double>(h) * 2.0, 360.0);
    const double ss = s / 255.0;
    const double vv = v / 255.0;
    const double c = vv * ss;
    const double x = c * (1.0 - std::fabs(std::fmod(hh / 60.0, 2.0) - 1.0));
    const double m = vv - c;
    double r = 0, g = 0, bl = 0;
    if (hh < 60) {
        r = c; g = x;
    } else if (hh < 120) {
        r = x; g = c;
    } else if (hh < 180) {
        g = c; bl = x;
    } else if (hh < 240) {
        g = x; bl = c;
    } else if (hh < 300) {
        r = x; bl = c;
    } else {
        r = c; bl = x;
    }
    return {static_cast<uint8_t>(std::lround((bl + m) * 255.0)),
            static_cast<uint8_t>(std::lround((g + m) * 255.0)),
            static_cast<uint8_t>(std::lround((r + m) * 255.0))};
}

void setPixel(Image& img, int x, int y, const Bgr& c) {
    if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
    uint8_t* p = img.pixel(x, y);
    p[0] = c.b;
    p[1] = c.g;
    p[2] = c.r;
}

// 以 (cx, cy) 为中心、绕 z 旋转 angleRad 的填充矩形
void fillRotatedRect(Image& img, double cx, double cy, double w, double h, double angleRad,
                     const Bgr& color) {
    const double c = std::cos(angleRad), s = std::sin(angleRad);
    const double r = 0.5 * std::sqrt(w * w + h * h) + 2.0;
    const int x0 = std::max(0, static_cast<int>(cx - r));
    const int x1 = std::min(img.width - 1, static_cast<int>(cx + r));
    const int y0 = std::max(0, static_cast<int>(cy - r));
    const int y1 = std::min(img.height - 1, static_cast<int>(cy + r));
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const double dx = x - cx, dy = y - cy;
            const double lx = c * dx + s * dy;
            const double ly = -s * dx + c * dy;
            if (std::fabs(lx) <= w * 0.5 && std::fabs(ly) <= h * 0.5) setPixel(img, x, y, color);
        }
    }
}

void addNoise(Image& img, double amplitude, std::mt19937& rng) {
    if (amplitude <= 0.0) return;
    const int amp = static_cast<int>(amplitude);
    for (size_t i = 0; i < img.data.size(); ++i) {
        const int n = static_cast<int>(rng() % static_cast<unsigned>(2 * amp + 1)) - amp;
        const int v = static_cast<int>(img.data[i]) + n;
        img.data[i] = static_cast<uint8_t>(std::clamp(v, 0, 255));
    }
}

}  // namespace

Image renderEmptyFrame(const RuneSceneConfig& cfg, std::mt19937& rng) {
    Image img(cfg.width, cfg.height);
    const Bgr bg{static_cast<uint8_t>(cfg.background), static_cast<uint8_t>(cfg.background),
                 static_cast<uint8_t>(cfg.background)};
    for (int y = 0; y < img.height; ++y)
        for (int x = 0; x < img.width; ++x) setPixel(img, x, y, bg);
    addNoise(img, cfg.noiseAmplitude, rng);
    return img;
}

Image renderRuneFrame(const RuneSceneConfig& cfg, double tSeconds, std::mt19937& rng) {
    Image img = renderEmptyFrame(cfg, rng);

    const double scale = cfg.scale;
    const double radius = cfg.ringRadiusPx * scale;
    const double bladeLen = 118.0 * scale;
    const double bladeW = 30.0 * scale;
    const double ledLen = 62.0 * scale;
    const double ledW = 16.0 * scale;

    const Bgr blade{static_cast<uint8_t>(cfg.bladeGray), static_cast<uint8_t>(cfg.bladeGray),
                    static_cast<uint8_t>(cfg.bladeGray)};

    // 闪烁：方波，占空比可调；待激活为常亮
    double level = 1.0;
    if (cfg.active) {
        const double phase = std::fmod(tSeconds * cfg.flickerHz, 1.0);
        const double duty = std::clamp(cfg.dutyCycle, 0.05, 0.95);
        level = phase < duty ? 1.0 : 0.25;
    }
    const int hue = cfg.active ? cfg.hueActive : cfg.hueInactive;
    const double base = cfg.active ? cfg.brightness : cfg.dimBrightness;
    const int v = static_cast<int>(std::lround(std::clamp(base * level, 0.0, 255.0)));
    const Bgr led = hsvToBgr(hue, cfg.saturation, v);

    for (int i = 0; i < 5; ++i) {
        const double ang = (cfg.rotationDeg + 72.0 * i) * kDeg2Rad;
        const double bx = cfg.centerX + std::cos(ang) * radius;
        const double by = cfg.centerY + std::sin(ang) * radius;
        fillRotatedRect(img, bx, by, bladeLen, bladeW, ang, blade);

        // LED 贴在叶片外侧
        const double lx = cfg.centerX + std::cos(ang) * (radius + bladeLen * 0.34);
        const double ly = cfg.centerY + std::sin(ang) * (radius + bladeLen * 0.34);
        fillRotatedRect(img, lx, ly, ledLen, ledW, ang, led);
    }

    // 中心靶面
    const double sq = cfg.targetSquarePx * scale;
    const Bgr plate{static_cast<uint8_t>(150), static_cast<uint8_t>(150),
                    static_cast<uint8_t>(150)};
    fillRotatedRect(img, cfg.centerX, cfg.centerY, sq, sq, 0.0, plate);
    const Bgr center{static_cast<uint8_t>(40), static_cast<uint8_t>(40), static_cast<uint8_t>(220)};
    fillRotatedRect(img, cfg.centerX, cfg.centerY, sq * 0.35, sq * 0.35, 0.0, center);

    addNoise(img, cfg.noiseAmplitude, rng);
    return img;
}

}  // namespace bff
