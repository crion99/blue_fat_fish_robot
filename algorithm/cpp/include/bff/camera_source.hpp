// 相机采集（OpenCV 后端）：支持 USB 相机与视频文件回放。
//
// 只在本机找到 OpenCV 时编译（CMake 中 BFF_WITH_OPENCV）。
// 采集线程与处理线程分离是后续优化项；当前版本在单线程里 read -> process -> imshow，
// 对 1280×720@60 的常规 USB 相机足够。

#pragma once

#ifdef BFF_WITH_OPENCV

#include <opencv2/videoio.hpp>

#include <memory>
#include <string>

#include "bff/image.hpp"

namespace bff {

struct CameraOptions {
    int index{0};
    std::string videoFile;  // 非空则从文件回放
    int width{1280};
    int height{720};
    double fps{60.0};
    int exposure{-1};  // <0 表示不修改（不同相机单位不同，按 SDK 说明设置）
    int gain{-1};
    std::string fourcc{"MJPG"};
};

class CameraSource {
public:
    CameraSource() = default;
    ~CameraSource();

    bool open(const CameraOptions& options, std::string& error);
    void close();

    // 返回 false 表示读帧失败（相机掉线或文件结束）
    bool read(Image& frame, double& timestampSec);

    bool isOpen() const { return open_; }
    double actualFps() const { return actualFps_; }
    const std::string& description() const { return description_; }

private:
    cv::VideoCapture capture_;
    bool open_{false};
    bool fromFile_{false};
    double actualFps_{0.0};
    std::string description_;
    double startSec_{0.0};
    long long frameIndex_{0};
};

}  // namespace bff

#endif  // BFF_WITH_OPENCV
