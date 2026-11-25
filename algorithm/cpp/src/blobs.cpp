#include "bff/blobs.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace bff {
namespace {

struct Accum {
    int minX{0}, minY{0}, maxX{-1}, maxY{-1};
    long long sumX{0}, sumY{0};
    long long count{0};
    double sumCos{0.0};
    double sumSin{0.0};
    double sumS{0.0};
    double sumV{0.0};
};

}  // namespace

std::vector<Blob> findLedBlobs(const Image& bgr, const HsvPlanes& hsv, int vMin, int sMin,
                               const BlobParams& params) {
    std::vector<Blob> blobs;
    if (bgr.empty() || hsv.width != bgr.width || hsv.height != bgr.height) return blobs;

    const int w = bgr.width;
    const int h = bgr.height;
    std::vector<uint8_t> visited(static_cast<size_t>(w) * h, 0);

    auto isMask = [&](int x, int y) {
        const size_t idx = static_cast<size_t>(y) * w + x;
        return hsv.v[idx] >= vMin && hsv.s[idx] >= sMin;
    };

    std::vector<int> stack;
    std::vector<Accum> accums;
    stack.reserve(4096);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t start = static_cast<size_t>(y) * w + x;
            if (visited[start] || !isMask(x, y)) continue;

            const int label = static_cast<int>(accums.size());
            Accum acc;
            acc.minX = acc.maxX = x;
            acc.minY = acc.maxY = y;
            accums.push_back(acc);

            stack.clear();
            stack.push_back(static_cast<int>(start));
            visited[start] = 1;

            while (!stack.empty()) {
                const int idx = stack.back();
                stack.pop_back();
                const int px = idx % w;
                const int py = idx / w;

                Accum& a = accums[label];
                a.count += 1;
                a.sumX += px;
                a.sumY += py;
                a.minX = std::min(a.minX, px);
                a.maxX = std::max(a.maxX, px);
                a.minY = std::min(a.minY, py);
                a.maxY = std::max(a.maxY, py);

                const size_t sidx = static_cast<size_t>(idx);
                const double hueRad = static_cast<double>(hsv.h[sidx]) * (2.0 * kPi / 180.0);
                a.sumCos += std::cos(hueRad);
                a.sumSin += std::sin(hueRad);
                a.sumS += hsv.s[sidx];
                a.sumV += hsv.v[sidx];

                const int nb[4][2] = {{px - 1, py}, {px + 1, py}, {px, py - 1}, {px, py + 1}};
                for (const auto& n : nb) {
                    const int nx = n[0], ny = n[1];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    const size_t nidx = static_cast<size_t>(ny) * w + nx;
                    if (visited[nidx] || !isMask(nx, ny)) continue;
                    visited[nidx] = 1;
                    stack.push_back(static_cast<int>(nidx));
                }
            }
        }
    }

    for (const auto& a : accums) {
        if (a.count < params.minArea || a.count > params.maxArea) continue;
        Blob b;
        b.area = static_cast<int>(a.count);
        b.box = {a.minX, a.minY, a.maxX - a.minX + 1, a.maxY - a.minY + 1};
        b.centroid = {static_cast<double>(a.sumX) / a.count, static_cast<double>(a.sumY) / a.count};
        double hueDeg = std::atan2(a.sumSin, a.sumCos) * kRad2Deg;
        if (hueDeg < 0.0) hueDeg += 360.0;
        b.meanH = hueDeg * 0.5;  // 回到 0..180
        b.meanS = a.sumS / a.count;
        b.meanV = a.sumV / a.count;

        const double aspect =
            static_cast<double>(std::max(b.box.w, b.box.h)) / std::max(1, std::min(b.box.w, b.box.h));
        if (aspect > params.maxAspect) continue;
        blobs.push_back(b);
    }

    std::sort(blobs.begin(), blobs.end(),
              [](const Blob& a, const Blob& b) { return a.area > b.area; });
    return blobs;
}

}  // namespace bff
