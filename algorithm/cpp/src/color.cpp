#include "bff/color.hpp"

#include <algorithm>
#include <cmath>

namespace bff {

HsvPlanes bgrToHsv(const Image& bgr) {
    HsvPlanes out;
    out.width = bgr.width;
    out.height = bgr.height;
    const size_t n = static_cast<size_t>(bgr.width) * bgr.height;
    out.h.assign(n, 0);
    out.s.assign(n, 0);
    out.v.assign(n, 0);
    if (bgr.empty()) return out;

    for (int y = 0; y < bgr.height; ++y) {
        const uint8_t* row = bgr.row(y);
        for (int x = 0; x < bgr.width; ++x) {
            const int b = row[3 * x + 0];
            const int g = row[3 * x + 1];
            const int r = row[3 * x + 2];
            const int mx = std::max({r, g, b});
            const int mn = std::min({r, g, b});
            const int delta = mx - mn;
            const size_t idx = static_cast<size_t>(y) * bgr.width + x;

            int h = 0;
            if (delta != 0) {
                if (mx == r) {
                    h = static_cast<int>(30.0 * (g - b) / delta + 0.5);  // 60°/2，映射到 0..180
                } else if (mx == g) {
                    h = static_cast<int>(30.0 * (b - r) / delta + 60.0 + 0.5);
                } else {
                    h = static_cast<int>(30.0 * (r - g) / delta + 120.0 + 0.5);
                }
                if (h < 0) h += 180;
                if (h >= 180) h -= 180;
            }
            const int s = mx == 0 ? 0 : (255 * delta) / mx;
            out.h[idx] = static_cast<uint8_t>(h);
            out.s[idx] = static_cast<uint8_t>(s);
            out.v[idx] = static_cast<uint8_t>(mx);
        }
    }
    return out;
}

int hueDistance(int a, int b) {
    int d = std::abs(a - b) % 180;
    return std::min(d, 180 - d);
}

}  // namespace bff
