#include "bff/image.hpp"

#include <algorithm>
#include <cstdio>

namespace bff {

bool Image::savePpm(const std::string& path) const {
    if (empty()) return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", width, height);
    // BGR -> RGB
    std::vector<uint8_t> rgb(data.size());
    for (size_t i = 0; i + 2 < data.size(); i += 3) {
        rgb[i + 0] = data[i + 2];
        rgb[i + 1] = data[i + 1];
        rgb[i + 2] = data[i + 0];
    }
    const size_t written = std::fwrite(rgb.data(), 1, rgb.size(), f);
    std::fclose(f);
    return written == rgb.size();
}

Image downscale(const Image& src, int factor) {
    if (src.empty() || factor <= 1) return src;
    const int w = src.width / factor;
    const int h = src.height / factor;
    if (w <= 0 || h <= 0) return Image{};
    Image out(w, h);
    const int n = factor * factor;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int b = 0, g = 0, r = 0;
            for (int dy = 0; dy < factor; ++dy) {
                const uint8_t* row = src.row(y * factor + dy);
                for (int dx = 0; dx < factor; ++dx) {
                    const uint8_t* p = row + 3 * (x * factor + dx);
                    b += p[0];
                    g += p[1];
                    r += p[2];
                }
            }
            uint8_t* dst = out.pixel(x, y);
            dst[0] = static_cast<uint8_t>(b / n);
            dst[1] = static_cast<uint8_t>(g / n);
            dst[2] = static_cast<uint8_t>(r / n);
        }
    }
    return out;
}

}  // namespace bff
