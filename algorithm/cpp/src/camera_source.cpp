#ifdef BFF_WITH_OPENCV

#include "bff/camera_source.hpp"

#include <chrono>
#include <cstdio>

namespace bff {
namespace {

double nowSeconds() {
    using clock = std::chrono::steady_clock;
    static const clock::time_point t0 = clock::now();
    return std::chrono::duration<double>(clock::now() - t0).count();
}

int fourccFromString(const std::string& s) {
    if (s.size() != 4) return 0;
    return cv::VideoWriter::fourcc(s[0], s[1], s[2], s[3]);
}

}  // namespace

CameraSource::~CameraSource() { close(); }

bool CameraSource::open(const CameraOptions& options, std::string& error) {
    close();
    error.clear();

    if (!options.videoFile.empty()) {
        fromFile_ = true;
        if (!capture_.open(options.videoFile, cv::CAP_ANY)) {
            error = "无法打开视频文件：" + options.videoFile;
            return false;
        }
        description_ = "file:" + options.videoFile;
    } else {
        fromFile_ = false;
        // Windows 下 DSHOW 打开更快、对 MJPG 支持更完整；其他平台用默认后端
#ifdef _WIN32
        const int backend = cv::CAP_DSHOW;
#else
        const int backend = cv::CAP_ANY;
#endif
        if (!capture_.open(options.index, backend)) {
            error = "无法打开相机 index=" + std::to_string(options.index);
            return false;
        }
        const int fourcc = fourccFromString(options.fourcc);
        if (fourcc != 0) capture_.set(cv::CAP_PROP_FOURCC, fourcc);
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
        capture_.set(cv::CAP_PROP_FPS, options.fps);
        if (options.exposure >= 0) capture_.set(cv::CAP_PROP_EXPOSURE, options.exposure);
        if (options.gain >= 0) capture_.set(cv::CAP_PROP_GAIN, options.gain);
        description_ = "camera#" + std::to_string(options.index);
    }

    actualFps_ = capture_.get(cv::CAP_PROP_FPS);
    startSec_ = nowSeconds();
    frameIndex_ = 0;
    open_ = true;

    char buf[128];
    std::snprintf(buf, sizeof(buf), " %dx%d @ %.1f fps", static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
                  static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)), actualFps_);
    description_ += buf;
    return true;
}

void CameraSource::close() {
    if (capture_.isOpened()) capture_.release();
    open_ = false;
}

bool CameraSource::read(Image& frame, double& timestampSec) {
    if (!open_) return false;
    cv::Mat mat;
    if (!capture_.read(mat) || mat.empty()) return false;

    if (mat.channels() == 3) {
        frame.width = mat.cols;
        frame.height = mat.rows;
        frame.data.resize(static_cast<size_t>(mat.cols) * mat.rows * 3);
        for (int y = 0; y < mat.rows; ++y) {
            const uint8_t* src = mat.ptr<uint8_t>(y);
            std::copy(src, src + static_cast<size_t>(mat.cols) * 3,
                      frame.data.begin() + static_cast<size_t>(y) * mat.cols * 3);
        }
    } else {
        cv::Mat bgr;
        cv::cvtColor(mat, bgr, cv::COLOR_GRAY2BGR);
        frame.width = bgr.cols;
        frame.height = bgr.rows;
        frame.data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.cols) * bgr.rows * 3);
    }

    if (fromFile_) {
        timestampSec = frameIndex_ / (actualFps_ > 1.0 ? actualFps_ : 30.0);
    } else {
        timestampSec = nowSeconds() - startSec_;
    }
    ++frameIndex_;
    return true;
}

}  // namespace bff

#endif  // BFF_WITH_OPENCV
