// 第二部分：相机采集 + 实时显示；同时把第一部分（灯效识别）和第三部分（位姿/转角）叠在画面上。
//
// 用法示例（Ubuntu）：
//   ./build/live_view --source 0
//   ./build/live_view --source /dev/video0 --width 1280 --height 720 --fps 60
//   ./build/live_view --source demo.mp4 --record out.avi
//   ./build/live_view --source 0 --pose          # 鼠标点 4 个角点 -> 打印位姿与瞄准转角
//
// 按键：q/ESC 退出、s 截图、p 切换点选测位姿、r 开始/结束录制。

#include <opencv2/opencv.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "bff/camera_source.hpp"
#include "bff/config.hpp"
#include "bff/pipeline.hpp"
#include "bff/rune_detector.hpp"

using namespace bff;

namespace {

constexpr const char* kWindow = "blue_fat_fish_algo";

struct Options {
    CameraOptions camera;
    std::string cameraConfig{"configs/camera.yml"};
    std::string gimbalConfig{"configs/gimbal.yml"};
    std::string targetConfig{"configs/target.yml"};
    std::string runeConfig{"configs/rune.yml"};
    std::string recordPath;
    std::string csvPath;  // 逐帧识别结果，供离线统计
    std::string snapshotDir{"snapshots"};
    double yawDeg{0.0};
    double pitchDeg{0.0};
    bool poseMode{false};
    bool headless{false};
};

struct UiState {
    std::vector<cv::Point2f> clicks;
    AimEstimate estimate;
    bool hasEstimate{false};
    std::string hint{"[p] pose: click 4 corners TL,TR,BR,BL"};
};

void onMouse(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto* state = static_cast<UiState*>(userdata);
    if (state->clicks.size() >= 4) state->clicks.clear();
    state->clicks.emplace_back(static_cast<float>(x), static_cast<float>(y));
}

Options parseArgs(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << name << " 需要一个参数\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--source") {
            const std::string value = next("--source");
            const bool isNumber = !value.empty() &&
                                  value.find_first_not_of("0123456789") == std::string::npos;
            if (isNumber) {
                opt.camera.index = std::atoi(value.c_str());
            } else {
                opt.camera.videoFile = value;
            }
        } else if (arg == "--width") {
            opt.camera.width = std::atoi(next("--width").c_str());
        } else if (arg == "--height") {
            opt.camera.height = std::atoi(next("--height").c_str());
        } else if (arg == "--fps") {
            opt.camera.fps = std::atof(next("--fps").c_str());
        } else if (arg == "--exposure") {
            opt.camera.exposure = std::atoi(next("--exposure").c_str());
        } else if (arg == "--gain") {
            opt.camera.gain = std::atoi(next("--gain").c_str());
        } else if (arg == "--camera-config") {
            opt.cameraConfig = next("--camera-config");
        } else if (arg == "--gimbal-config") {
            opt.gimbalConfig = next("--gimbal-config");
        } else if (arg == "--target-config") {
            opt.targetConfig = next("--target-config");
        } else if (arg == "--rune-config") {
            opt.runeConfig = next("--rune-config");
        } else if (arg == "--record") {
            opt.recordPath = next("--record");
        } else if (arg == "--csv") {
            opt.csvPath = next("--csv");
        } else if (arg == "--snapshot-dir") {
            opt.snapshotDir = next("--snapshot-dir");
        } else if (arg == "--yaw") {
            opt.yawDeg = std::atof(next("--yaw").c_str());
        } else if (arg == "--pitch") {
            opt.pitchDeg = std::atof(next("--pitch").c_str());
        } else if (arg == "--pose") {
            opt.poseMode = true;
        } else if (arg == "--headless") {
            opt.headless = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "live_view --source <index|video> [--width N --height N --fps N]\n"
                         "          [--camera-config f --gimbal-config f --target-config f --rune-config f]\n"
                         "          [--record out.avi] [--pose] [--yaw deg --pitch deg] [--headless]\n";
            std::exit(0);
        } else {
            std::cerr << "未知参数：" << arg << "（--help 查看用法）\n";
            std::exit(2);
        }
    }
    return opt;
}

cv::Scalar colorFor(RuneState state) {
    switch (state) {
        case RuneState::Active:
            return {0, 0, 255};
        case RuneState::Inactive:
            return {255, 180, 0};
        default:
            return {160, 160, 160};
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parseArgs(argc, argv);

    Intrinsics intrinsics;
    GimbalConfig gimbal;
    TargetModel target;
    RuneConfig runeConfig;
    std::string err;
    if (!opt.cameraConfig.empty() && !loadCameraConfig(opt.cameraConfig, intrinsics, err)) {
        std::cerr << "相机配置：" << err << "\n";
    }
    if (!opt.gimbalConfig.empty() && !loadGimbalConfig(opt.gimbalConfig, gimbal, err)) {
        std::cerr << "云台配置：" << err << "\n";
    }
    if (!opt.targetConfig.empty() && !loadTargetConfig(opt.targetConfig, target, err)) {
        std::cerr << "靶标配置：" << err << "\n";
    }
    if (!opt.runeConfig.empty() && !loadRuneConfig(opt.runeConfig, runeConfig, err)) {
        std::cerr << "灯效配置：" << err << "\n";
    }

    CameraSource source;
    if (!source.open(opt.camera, err)) {
        std::cerr << err << "\n";
        return 1;
    }
    std::cout << "已打开 " << source.description() << "\n";

    RuneDetector detector(runeConfig);
    PoseEstimator estimator(intrinsics, target, gimbal);

    UiState ui;
    if (!opt.headless) {
        cv::namedWindow(kWindow, cv::WINDOW_NORMAL);
        cv::setMouseCallback(kWindow, onMouse, &ui);
    }

    cv::VideoWriter writer;
    bool recording = false;
    auto startRecording = [&](int width, int height) {
        if (opt.recordPath.empty()) return;
        writer.open(opt.recordPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0,
                    cv::Size(width, height));
        recording = writer.isOpened();
        std::cout << (recording ? "开始录制 " : "录制失败 ") << opt.recordPath << "\n";
    };

    std::deque<double> frameTimes;
    std::ofstream csv;
    if (!opt.csvPath.empty()) {
        csv.open(opt.csvPath);
        csv << "frame,t,state,candidate,score,detected,x,y,w,h,meanV,blinkHz,modulation\n";
    }
    double yawCurrent = opt.yawDeg * kDeg2Rad;
    double pitchCurrent = opt.pitchDeg * kDeg2Rad;
    long long frameCount = 0;
    bool running = true;
    int snapshotIndex = 0;

    while (running) {
        Image frame;
        double timestamp = 0.0;
        if (!source.read(frame, timestamp)) break;

        const RuneObservation obs = detector.process(frame, timestamp);
        cv::Mat view(frame.height, frame.width, CV_8UC3, frame.data.data());
        if (csv.is_open()) {
            csv << frameCount << ',' << timestamp << ',' << runeStateName(obs.state) << ','
                << runeStateName(obs.candidate) << ',' << obs.scoreActive << ','
                << (obs.detected ? 1 : 0) << ',' << obs.box.x << ',' << obs.box.y << ','
                << obs.box.w << ',' << obs.box.h << ',' << obs.features.meanV << ','
                << obs.features.blinkHz << ',' << obs.features.modulationDepth << '\n';
        }
        if (frameCount == 0 && !opt.recordPath.empty()) {
            startRecording(frame.width, frame.height);
        }

        // ---- 第一部分：灯效识别标注 ----
        if (obs.detected) {
            const cv::Rect box(obs.box.x, obs.box.y, obs.box.w, obs.box.h);
            cv::rectangle(view, box, colorFor(obs.state), 3);
            const std::string label =
                std::string(runeStateName(obs.state)) + " " +
                std::to_string(static_cast<int>(obs.confidence * 100.0)) + "%";
            cv::putText(view, label, cv::Point(box.x, std::max(24, box.y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, colorFor(obs.state), 2);
            char feat[160];
            std::snprintf(feat, sizeof(feat), "V=%.0f H=%.0f blink=%.1fHz mod=%.2f",
                          obs.features.meanV, obs.features.meanH, obs.features.blinkHz,
                          obs.features.modulationDepth);
            cv::putText(view, feat, cv::Point(box.x, box.y + box.height + 24),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {200, 255, 200}, 2);
        }

        // ---- 第三部分：鼠标点 4 角 -> 位姿 + 瞄准转角 ----
        if (opt.poseMode) {
            for (const auto& p : ui.clicks) cv::circle(view, p, 6, {0, 255, 255}, -1);
            if (ui.clicks.size() == 4) {
                std::array<Vec2, 4> corners{};
                for (int i = 0; i < 4; ++i) {
                    corners[i] = {static_cast<double>(ui.clicks[i].x),
                                  static_cast<double>(ui.clicks[i].y)};
                }
                ui.estimate = estimator.aim(corners, yawCurrent, pitchCurrent);
                ui.hasEstimate = true;
                for (int i = 0; i < 4; ++i) {
                    cv::line(view, ui.clicks[i], ui.clicks[(i + 1) % 4], {0, 255, 0}, 2);
                }
            }
            if (ui.hasEstimate) {
                const AimSolution& a = ui.estimate.aim;
                char line1[200];
                std::snprintf(line1, sizeof(line1),
                              "dYaw=%+.2f deg dPitch=%+.2f deg dist=%.2f m rms=%.2f px %s",
                              a.deltaYaw * kRad2Deg, a.deltaPitch * kRad2Deg, a.distance,
                              ui.estimate.pose.reprojRmsPx,
                              ui.estimate.accepted ? "OK" : "REJ");
                cv::putText(view, line1, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            {0, 255, 0}, 2);
                char line2[200];
                std::snprintf(line2, sizeof(line2), "yaw=%.2f pitch=%.2f cmd: %s",
                              a.yaw * kRad2Deg, a.pitch * kRad2Deg,
                              PoseEstimator::formatCommand(ui.estimate).c_str());
                cv::putText(view, line2, cv::Point(20, 72), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            {0, 255, 0}, 2);
            }
        }

        // ---- 帧率 ----
        frameTimes.push_back(timestamp);
        while (frameTimes.size() > 60) frameTimes.pop_front();
        double fps = 0.0;
        if (frameTimes.size() >= 2) {
            const double dt = frameTimes.back() - frameTimes.front();
            if (dt > 1e-6) fps = static_cast<double>(frameTimes.size() - 1) / dt;
        }
        char hud[160];
        std::snprintf(hud, sizeof(hud), "fps %.1f | frame %lld | %s", fps, frameCount,
                      opt.poseMode ? ui.hint.c_str() : "[p] pose  [s] shot  [r] rec  [q] quit");
        cv::putText(view, hud, cv::Point(20, view.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    {0, 255, 0}, 2);

        if (recording) writer.write(view);

        if (!opt.headless) {
            cv::imshow(kWindow, view);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 27) {
                running = false;
            } else if (key == 'p') {
                opt.poseMode = !opt.poseMode;
                ui.clicks.clear();
                ui.hasEstimate = false;
            } else if (key == 's') {
                const std::string path = opt.snapshotDir + "/snap_" + std::to_string(snapshotIndex++) + ".png";
                cv::imwrite(path, view);
                std::cout << "截图：" << path << "\n";
            } else if (key == 'r') {
                if (recording) {
                    writer.release();
                    recording = false;
                    std::cout << "停止录制\n";
                } else {
                    startRecording(view.cols, view.rows);
                }
            }
        } else if (frameCount % 60 == 0) {
            std::cout << "frame " << frameCount << " fps " << fps << " state "
                      << runeStateName(obs.state) << "\n";
        }
        ++frameCount;
    }

    if (recording) writer.release();
    std::cout << "共处理 " << frameCount << " 帧\n";
    return 0;
}
