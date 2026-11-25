// 配置读取（OpenCV cv::FileStorage，支持 YAML / JSON）。
//
// 所有阈值、内参、外参都在 configs/*.yml 里，改参数不需要重新编译。

#pragma once

#ifdef BFF_WITH_OPENCV

#include <string>

#include "bff/aim.hpp"
#include "bff/camera.hpp"
#include "bff/pipeline.hpp"
#include "bff/rune_detector.hpp"

namespace bff {

bool loadCameraConfig(const std::string& path, Intrinsics& out, std::string& error);
bool loadGimbalConfig(const std::string& path, GimbalConfig& out, std::string& error);
bool loadTargetConfig(const std::string& path, TargetModel& out, std::string& error);
bool loadRuneConfig(const std::string& path, RuneConfig& out, std::string& error);

}  // namespace bff

#endif  // BFF_WITH_OPENCV
