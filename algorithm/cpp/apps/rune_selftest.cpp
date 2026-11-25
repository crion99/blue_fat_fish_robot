// 第一部分自检：合成灯效序列 -> 灯效识别 -> 逐帧/逐段准确率、稳定延迟、误检率。
//
// 用法：rune_selftest [输出目录] [每类片段数]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "bff/annotate.hpp"
#include "bff/rune_detector.hpp"
#include "bff/synthetic_rune.hpp"

using namespace bff;

namespace {

constexpr int kFrames = 90;
constexpr double kFps = 30.0;
constexpr int kWarmup = 20;

struct ClipResult {
    RuneState truth{RuneState::Unknown};
    std::vector<RuneState> states;
    std::vector<double> scores;
    std::vector<bool> detected;
    int latencyFrames{-1};
    double processMs{0.0};
    RuneState predicted{RuneState::Unknown};
    bool detectedAny{false};
};

RuneSceneConfig makeConfig(std::mt19937& rng, bool active) {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    RuneSceneConfig cfg;
    cfg.centerX = 400.0 + 480.0 * uni(rng);
    cfg.centerY = 220.0 + 260.0 * uni(rng);
    cfg.ringRadiusPx = 70.0 + 80.0 * uni(rng);
    cfg.scale = 0.7 + 0.7 * uni(rng);
    cfg.rotationDeg = 360.0 * uni(rng);
    cfg.active = active;
    cfg.brightness = active ? (200.0 + 55.0 * uni(rng)) : 0.0;
    cfg.dimBrightness = 110.0 + 60.0 * uni(rng);
    cfg.flickerHz = active ? (1.5 + 6.5 * uni(rng)) : 0.0;
    cfg.dutyCycle = active ? (0.35 + 0.3 * uni(rng)) : 1.0;
    cfg.noiseAmplitude = 2.0 + 5.0 * uni(rng);
    cfg.background = 15.0 + 20.0 * uni(rng);
    return cfg;
}

ClipResult runClip(const RuneSceneConfig& cfg, bool empty, std::mt19937& rng) {
    ClipResult res;
    res.truth = empty ? RuneState::Unknown : (cfg.active ? RuneState::Active : RuneState::Inactive);

    RuneDetector detector;
    for (int f = 0; f < kFrames; ++f) {
        const double t = f / kFps;
        const Image frame = empty ? renderEmptyFrame(cfg, rng) : renderRuneFrame(cfg, t, rng);
        const auto t0 = std::chrono::steady_clock::now();
        const RuneObservation obs = detector.process(frame, t);
        const auto t1 = std::chrono::steady_clock::now();
        res.processMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
        res.states.push_back(obs.state);
        res.scores.push_back(obs.scoreActive);
        res.detected.push_back(obs.detected);
        if (obs.detected) res.detectedAny = true;
    }

    // 稳定延迟：从某帧起状态等于真值且一直保持到结尾
    if (!empty) {
        for (int f = 0; f < kFrames; ++f) {
            bool stable = true;
            for (int k = f; k < kFrames; ++k) {
                if (res.states[k] != res.truth) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                res.latencyFrames = f;
                break;
            }
        }
        // 预测：后 1/3 帧的多数票
        int votesActive = 0, votesInactive = 0;
        for (int f = kFrames * 2 / 3; f < kFrames; ++f) {
            if (res.states[f] == RuneState::Active) ++votesActive;
            if (res.states[f] == RuneState::Inactive) ++votesInactive;
        }
        res.predicted = votesActive > votesInactive ? RuneState::Active : RuneState::Inactive;
    }
    return res;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string outDir = argc > 1 ? argv[1] : "out";
    const int clipsPerClass = argc > 2 ? std::atoi(argv[2]) : 20;

    std::mt19937 rng(20261125u);

    std::vector<ClipResult> activeClips, inactiveClips, emptyClips;
    for (int i = 0; i < clipsPerClass; ++i) {
        activeClips.push_back(runClip(makeConfig(rng, true), false, rng));
        inactiveClips.push_back(runClip(makeConfig(rng, false), false, rng));
    }
    for (int i = 0; i < std::max(5, clipsPerClass / 2); ++i) {
        RuneSceneConfig cfg;
        cfg.noiseAmplitude = 2.0 + 5.0 * (static_cast<double>(rng() % 1000) / 1000.0);
        emptyClips.push_back(runClip(cfg, true, rng));
    }

    // 逐帧准确率（跳过前 kWarmup 帧）
    long long frames = 0, correctFrames = 0;
    for (const auto& c : activeClips)
        for (int f = kWarmup; f < kFrames; ++f) {
            ++frames;
            correctFrames += c.states[f] == RuneState::Active ? 1 : 0;
        }
    for (const auto& c : inactiveClips)
        for (int f = kWarmup; f < kFrames; ++f) {
            ++frames;
            correctFrames += c.states[f] == RuneState::Inactive ? 1 : 0;
        }
    const double frameAcc = frames ? static_cast<double>(correctFrames) / frames : 0.0;

    int clipCorrect = 0, clipTotal = 0;
    double latencySum = 0.0;
    int latencyCount = 0;
    for (const auto& c : activeClips) {
        ++clipTotal;
        clipCorrect += c.predicted == RuneState::Active ? 1 : 0;
        if (c.latencyFrames >= 0) {
            latencySum += c.latencyFrames;
            ++latencyCount;
        }
    }
    for (const auto& c : inactiveClips) {
        ++clipTotal;
        clipCorrect += c.predicted == RuneState::Inactive ? 1 : 0;
        if (c.latencyFrames >= 0) {
            latencySum += c.latencyFrames;
            ++latencyCount;
        }
    }
    const double clipAcc = clipTotal ? static_cast<double>(clipCorrect) / clipTotal : 0.0;

    int fpClips = 0;
    long long fpFrames = 0, emptyFrames = 0;
    for (const auto& c : emptyClips) {
        bool any = false;
        for (bool d : c.detected) any = any || d;
        if (any) ++fpClips;
        for (int f = kWarmup; f < kFrames; ++f) {
            ++emptyFrames;
            fpFrames += c.states[f] != RuneState::Unknown ? 1 : 0;
        }
    }

    double totalMs = 0.0;
    int totalClips = 0;
    for (const auto& c : activeClips) {
        totalMs += c.processMs;
        ++totalClips;
    }
    for (const auto& c : inactiveClips) {
        totalMs += c.processMs;
        ++totalClips;
    }
    for (const auto& c : emptyClips) {
        totalMs += c.processMs;
        ++totalClips;
    }
    const double msPerFrame = totalClips ? totalMs / (totalClips * kFrames) : 0.0;

    std::printf("clips: active=%d inactive=%d empty=%d\n", static_cast<int>(activeClips.size()),
                static_cast<int>(inactiveClips.size()), static_cast<int>(emptyClips.size()));
    std::printf("frame accuracy (after warmup) = %.2f%%\n", frameAcc * 100.0);
    std::printf("clip accuracy = %.2f%% (%d/%d)\n", clipAcc * 100.0, clipCorrect, clipTotal);
    std::printf("mean stabilization latency = %.2f frames (%.0f ms)\n",
                latencyCount ? latencySum / latencyCount : 0.0,
                1000.0 * (latencyCount ? latencySum / latencyCount : 0.0) / kFps);
    std::printf("detector cost = %.2f ms/frame (%.0f fps capacity, single thread)\n", msPerFrame,
                msPerFrame > 0.0 ? 1000.0 / msPerFrame : 0.0);
    std::printf("false detection: %d/%d empty clips, %lld/%lld frames\n", fpClips,
                static_cast<int>(emptyClips.size()), fpFrames, emptyFrames);

    // 标注示例：取一个激活片段和一个待激活片段的最后一帧
    auto annotateLast = [&](const RuneSceneConfig& cfg, bool active, const std::string& path) {
        std::mt19937 local(7u);
        RuneSceneConfig c = cfg;
        c.active = active;
        RuneDetector detector;
        Image frame;
        RuneObservation obs;
        for (int f = 0; f < kFrames; ++f) {
            frame = renderRuneFrame(c, f / kFps, local);
            obs = detector.process(frame, f / kFps);
        }
        const uint8_t r = active ? 0 : 255, g = active ? 60 : 200, b = active ? 255 : 40;
        drawRect(frame, obs.box, b, g, r, 3);
        for (const auto& bb : obs.blobBoxes) drawRect(frame, bb, 0, 255, 255, 1);
        char label[96];
        std::snprintf(label, sizeof(label), "%s %.0f%%", runeStateName(obs.state),
                      obs.confidence * 100.0);
        drawText(frame, obs.box.x, std::max(20, obs.box.y - 8), label, b, g, r, 2);
        frame.savePpm(path);
    };

    RuneSceneConfig sample;
    sample.centerX = 640;
    sample.centerY = 360;
    sample.ringRadiusPx = 120;
    sample.scale = 1.0;
    sample.noiseAmplitude = 3.0;
    sample.flickerHz = 4.0;
    sample.brightness = 235;
    sample.dimBrightness = 140;
    annotateLast(sample, true, outDir + "/annotated_active.ppm");
    annotateLast(sample, false, outDir + "/annotated_inactive.ppm");

    // 报告
    std::ofstream rep(outDir + "/report.md");
    rep << "# 灯效识别（待激活 / 已激活）合成数据验证\n\n";
    rep << "- 程序：`algorithm/cpp/apps/rune_selftest.cpp`\n";
    rep << "- 每类片段：" << clipsPerClass << "，每段 " << kFrames << " 帧 @ " << kFps << " fps\n";
    rep << "- 前 " << kWarmup << " 帧为预热（填满 1.5 s 时域窗口），不计入逐帧准确率\n\n";
    rep << "| 指标 | 结果 |\n| --- | --- |\n";
    rep << "| 逐帧准确率 | " << frameAcc * 100.0 << "% |\n";
    rep << "| 片段准确率 | " << clipAcc * 100.0 << "% (" << clipCorrect << "/" << clipTotal
        << ") |\n";
    rep << "| 平均稳定延迟 | " << (latencyCount ? latencySum / latencyCount : 0.0) << " 帧 / "
        << 1000.0 * (latencyCount ? latencySum / latencyCount : 0.0) / kFps << " ms |\n";
    rep << "| 空场景误检 | " << fpClips << "/" << emptyClips.size() << " 段，"
        << (emptyFrames ? 100.0 * static_cast<double>(fpFrames) / emptyFrames : 0.0) << "% 帧 |\n\n";
    rep << "## 判别规则\n\n";
    rep << "score = 0.45·[激活色相占比≥阈值] + 0.25·[亮度调制深度≥阈值] + "
           "0.15·[闪烁频率在区间内] + 0.15·[平均亮度≥阈值]，score ≥ 0.5 判为已激活，"
           "并需连续 5 帧一致才切换状态。\n\n";
    rep << "合成数据的两种状态在色相与闪烁上完全可分，因此准确率接近 100% 只说明"
           "**实现与时域逻辑正确**；真实场景的阈值必须用实拍片段重新标定（README 第 4 节）。\n";
    return 0;
}


