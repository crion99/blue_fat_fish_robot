#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <fstream>
#include <iomanip>
#include <locale>
#include <optional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "robomini_interfaces/msg/gimbal_to_vision.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>

class LightDetector : public rclcpp::Node
{
public:
    LightDetector()
        : Node("energy_light_detector")
    {
        // 声明参数
        const auto csv_path = declare_parameter<std::string>("pose_csv_path", "");
        const auto diagnostic_path = declare_parameter<std::string>("diagnostic_csv_path", "");
        feedback_timeout_ms_ = declare_parameter<double>("feedback_timeout_ms", 250.0);
        prefer_frontal_pose_ = declare_parameter<bool>("prefer_frontal_pose", true);

        if (!std::isfinite(feedback_timeout_ms_) || feedback_timeout_ms_ <= 0.0)
            throw std::invalid_argument("feedback_timeout_ms must be finite and positive");

        // 初始化 CSV
        if (!diagnostic_path.empty())
        {
            diagnostic_csv_.open(diagnostic_path, std::ios::app);
            if (!diagnostic_csv_)
                throw std::runtime_error("Cannot open diagnostic CSV: " + diagnostic_path);
            diagnostic_csv_.imbue(std::locale::classic());
            diagnostic_csv_ << std::setprecision(12);
            if (diagnostic_csv_.tellp() == 0)
            {
                diagnostic_csv_ << "timestamp,target_detected,pixel_x,pixel_y,camera_x,camera_y,camera_z,"
                                   "reprojection_rmse,yaw,pitch,yaw_vel,pitch_vel,feedback_age_ms,"
                                   "pose_timestamp,feedback_receipt_timestamp,receipt_minus_image_ms,"
                                   "pose_valid,pose_status,feedback_status\n";
            }
        }

        if (!csv_path.empty())
        {
            pose_csv_.open(csv_path, std::ios::app);
            if (!pose_csv_)
                throw std::runtime_error("Cannot open pose CSV: " + csv_path);
            pose_csv_.imbue(std::locale::classic());
            if (pose_csv_.tellp() == 0)
            {
                pose_csv_ << "stamp_sec,stamp_nanosec,status,u1,v1,u2,v2,u3,v3,u4,v4,"
                             "x_right_m,y_down_m,z_forward_m,reprojection_rmse_px\n";
            }
        }

        // 订阅相机
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&LightDetector::image_callback, this, std::placeholders::_1));

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
                camera_info_ = std::move(info);
            });

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/vision/pending_target_pose", rclcpp::SensorDataQoS());

        state_pub_ = create_publisher<std_msgs::msg::String>("/vision/rune_states", 10);

        feedback_sub_ = create_subscription<robomini_interfaces::msg::GimbalToVision>(
            "/gimbal_to_vision", rclcpp::SensorDataQoS(),
            [this](robomini_interfaces::msg::GimbalToVision::ConstSharedPtr message) {
                const auto receipt = now();
                feedback_ = Feedback{receipt, std::chrono::steady_clock::now(),
                    get_clock()->ros_time_is_active() && receipt.nanoseconds() > 0,
                    message->yaw, message->pitch, message->yaw_vel, message->pitch_vel};
            });

        RCLCPP_INFO(this->get_logger(), "Energy light detector started!");
        RCLCPP_INFO(this->get_logger(), "Waiting for camera images...");
    }

private:
    struct LightPair {
        cv::Point2d center;
        double separation;
        cv::Point2d first;
        cv::Point2d second;
    };

    struct PoseMeasurement {
        std::vector<cv::Point2d> image_points;
        cv::Vec3d translation;
        cv::Matx33d rotation;
        double rmse;
    };

    struct Feedback {
        rclcpp::Time receipt;
        std::chrono::steady_clock::time_point steady_receipt;
        bool has_sim_receipt;
        double yaw;
        double pitch;
        double yaw_vel;
        double pitch_vel;
    };

    static std::string stamp_text(const builtin_interfaces::msg::Time& stamp) {
        return cv::format("%d.%09u", stamp.sec, stamp.nanosec);
    }

    void show_diagnostics(cv::Mat& frame, const sensor_msgs::msg::Image& image,
        const std::optional<cv::Point>& target, const std::optional<PoseMeasurement>& pose,
        const char* pose_status)
    {
        const std::string image_stamp = stamp_text(image.header.stamp);
        const std::string receipt_stamp = feedback_ ? stamp_text(static_cast<builtin_interfaces::msg::Time>(feedback_->receipt)) : "";
        std::optional<double> offset_ms;
        double age_ms = 0.0;
        const char* feedback_status = "NO FEEDBACK";

        if (feedback_) {
            age_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - feedback_->steady_receipt).count();
            if (feedback_->has_sim_receipt && get_clock()->ros_time_is_active() && now() >= feedback_->receipt) {
                const rclcpp::Time image_time(image.header.stamp, RCL_ROS_TIME);
                if (image_time.nanoseconds() != 0)
                    offset_ms = (feedback_->receipt - image_time).seconds() * 1000.0;
            }
            const bool finite = std::isfinite(feedback_->yaw) && std::isfinite(feedback_->pitch) && std::isfinite(feedback_->yaw_vel) && std::isfinite(feedback_->pitch_vel);
            feedback_status = !finite ? "INVALID" : age_ms > feedback_timeout_ms_ ? "STALE" : "RECENT RECEIPT";
        }

        const cv::Scalar white(245, 245, 245);
        const cv::Scalar background(35, 35, 35);
        auto text = [&](const std::string& line, cv::Point pt) {
            cv::putText(frame, line, pt, cv::FONT_HERSHEY_SIMPLEX, 0.52, white, 1, cv::LINE_AA);
        };

        const std::string feedback_heading = std::string("Gimbal | ") + feedback_status;
        const std::string angles = feedback_ ? cv::format("Yaw %.3f  Pitch %.3f rad", feedback_->yaw, feedback_->pitch) : "";
        const std::string velocities = feedback_ ? cv::format("Yaw vel %.3f  Pitch vel %.3f rad/s", feedback_->yaw_vel, feedback_->pitch_vel) : "";
        
        int panel_width = 0;
        for (const auto& line : {feedback_heading, angles, velocities}) {
            int baseline = 0;
            const auto size = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.52, 1, &baseline);
            panel_width = std::max(panel_width, size.width + 20);
        }
        const int right = std::max(0, frame.cols - panel_width);
        cv::rectangle(frame, cv::Point(right, 0), cv::Point(frame.cols - 1, 86), background, cv::FILLED);
        text(feedback_heading, {right + 10, 21});
        if (feedback_) {
            text(angles, {right + 10, 44});
            text(velocities, {right + 10, 67});
        }

        const int footer = std::max(0, frame.rows - 78);
        cv::rectangle(frame, cv::Point(0, footer), cv::Point(frame.cols - 1, frame.rows - 1), background, cv::FILLED);
        text(std::string(target ? "PENDING" : "LOST") + (pose ? " | Pose valid" : " | Pose invalid: ") + (pose ? "" : pose_status), {12, footer + 19});
        text("Image " + image_stamp + "  |  Pose " + (pose ? image_stamp : "--") + "  |  Feedback receipt " + (feedback_ ? receipt_stamp : "--"), {12, footer + 42});
        text((feedback_ ? cv::format("Feedback receipt age %.1f ms", age_ms) : std::string("Feedback receipt age --")) + "  |  " + (offset_ms ? cv::format("Receipt minus image %+.1f ms (approx.)", *offset_ms) : std::string("Receipt minus image unavailable")), {12, footer + 65});

        if (!diagnostic_csv_.is_open()) return;
        diagnostic_csv_ << image_stamp << ',' << (target ? 1 : 0);
        if (target) diagnostic_csv_ << ',' << target->x << ',' << target->y; else diagnostic_csv_ << ",,";
        if (pose) diagnostic_csv_ << ',' << pose->translation[0] << ',' << pose->translation[1] << ',' << pose->translation[2] << ',' << pose->rmse; else diagnostic_csv_ << ",,,,";
        if (feedback_) diagnostic_csv_ << ',' << feedback_->yaw << ',' << feedback_->pitch << ',' << feedback_->yaw_vel << ',' << feedback_->pitch_vel << ',' << age_ms; else diagnostic_csv_ << ",,,,,";
        diagnostic_csv_ << ',' << (pose ? image_stamp : "") << ',' << receipt_stamp << ',';
        if (offset_ms) diagnostic_csv_ << *offset_ms;
        diagnostic_csv_ << ',' << (pose ? 1 : 0) << ',' << pose_status << ',' << feedback_status << '\n' << std::flush;
        if (!diagnostic_csv_) {
            RCLCPP_ERROR(get_logger(), "Failed writing diagnostic CSV; logging stopped");
            diagnostic_csv_.close();
        }
    }

    std::optional<PoseMeasurement> draw_pose_reprojection(cv::Mat& frame, const LightPair& radial, const LightPair& tangential, const sensor_msgs::msg::CameraInfo& info)
    {
        const cv::Matx33d intrinsics(info.k.data());
        const auto& distortion = info.d;
        constexpr double half_spacing = 0.0475;
        const std::vector<cv::Point3d> object_points{
            {-half_spacing, 0.0, 0.0}, {half_spacing, 0.0, 0.0},
            {0.0, -half_spacing, 0.0}, {0.0, half_spacing, 0.0}
        };
        const cv::Point2d tangent(radial.first.y - radial.second.y, radial.second.x - radial.first.x);
        const bool tangent_in_order = (tangential.second - tangential.first).dot(tangent) > 0.0;
        const std::vector<cv::Point2d> image_points{
            radial.first, radial.second,
            tangent_in_order ? tangential.first : tangential.second,
            tangent_in_order ? tangential.second : tangential.first
        };

        std::vector<cv::Vec3d> rotations;
        std::vector<cv::Vec3d> translations;
        cv::solvePnPGeneric(object_points, image_points, intrinsics, distortion, rotations, translations, false, cv::SOLVEPNP_IPPE);

        struct PoseCandidate {
            std::vector<cv::Point2d> projection;
            cv::Vec3d translation;
            cv::Matx33d rotation;
            double error;
            double optical_alignment;
        };
        std::vector<PoseCandidate> candidates;
        
        for (std::size_t i = 0; i < rotations.size(); ++i) {
            cv::Matx33d rotation;
            cv::Rodrigues(rotations[i], rotation);
            if (std::any_of(object_points.begin(), object_points.end(), [&](const cv::Point3d& point) {
                return (rotation * cv::Vec3d(point) + translations[i])[2] <= 0.0;
            })) continue;

            std::vector<cv::Point2d> projection;
            cv::projectPoints(object_points, rotations[i], translations[i], intrinsics, distortion, projection);
            const double error = cv::norm(image_points, projection) / std::sqrt(image_points.size());
            if (std::isfinite(error))
                candidates.push_back({projection, translations[i], rotation, error, std::abs(rotation(2, 2))});
        }

        if (candidates.empty()) return std::nullopt;

        auto selected = std::min_element(candidates.begin(), candidates.end(), [](const PoseCandidate& a, const PoseCandidate& b) { return a.error < b.error; });
        if (prefer_frontal_pose_) {
            const double error_limit = selected->error + 0.5;
            for (auto candidate = candidates.begin(); candidate != candidates.end(); ++candidate) {
                if (candidate->error <= error_limit && candidate->optical_alignment > selected->optical_alignment)
                    selected = candidate;
            }
        }

        const cv::Scalar color(255, 255, 0);
        for (std::size_t i = 0; i < image_points.size(); ++i) {
            const cv::Point measured(image_points[i]);
            const cv::Point projected(selected->projection[i]);
            cv::drawMarker(frame, projected, color, cv::MARKER_CROSS, 12, 2);
            cv::line(frame, measured, projected, color, 1);
            cv::putText(frame, std::to_string(i + 1), measured + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        cv::putText(frame, cv::format("Reprojection RMSE: %.2f px", selected->error), cv::Point(20, 185), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        const auto& position = selected->translation;
        cv::putText(frame, cv::format("Camera XYZ: (%.3f, %.3f, %.3f) m", position[0], position[1], position[2]), cv::Point(20, 215), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        const double tilt_deg = std::acos(std::clamp(selected->optical_alignment, 0.0, 1.0)) * 180.0 / CV_PI;
        cv::putText(frame, cv::format("Plate Tilt: %.1f deg", tilt_deg), cv::Point(20, 245), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

        return PoseMeasurement{image_points, selected->translation, selected->rotation, selected->error};
    }

    void publish_pose(const sensor_msgs::msg::Image& image, const PoseMeasurement& measurement)
    {
        geometry_msgs::msg::PoseStamped message;
        message.header.stamp = image.header.stamp;
        message.header.frame_id = image.header.frame_id + "_optical_frame";
        message.pose.position.x = measurement.translation[0];
        message.pose.position.y = measurement.translation[1];
        message.pose.position.z = measurement.translation[2];

        const auto& r = measurement.rotation;
        tf2::Quaternion orientation;
        tf2::Matrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2)).getRotation(orientation);
        orientation.normalize();
        message.pose.orientation.x = orientation.x();
        message.pose.orientation.y = orientation.y();
        message.pose.orientation.z = orientation.z();
        message.pose.orientation.w = orientation.w();
        pose_pub_->publish(message);
    }

    void write_pose_row(const sensor_msgs::msg::Image& image, const std::optional<PoseMeasurement>& measurement, const char* status)
    {
        if (!pose_csv_.is_open()) return;
        pose_csv_ << image.header.stamp.sec << ',' << image.header.stamp.nanosec << ',' << status;
        if (measurement) {
            pose_csv_ << std::setprecision(12);
            for (const auto& pixel : measurement->image_points) pose_csv_ << ',' << pixel.x << ',' << pixel.y;
            const auto& t = measurement->translation;
            pose_csv_ << ',' << t[0] << ',' << t[1] << ',' << t[2] << ',' << measurement->rmse;
        } else {
            pose_csv_ << ",,,,,,,,,,,,";
        }
        pose_csv_ << '\n' << std::flush;
        if (!pose_csv_) {
            RCLCPP_ERROR(get_logger(), "Failed writing pose CSV; logging stopped");
            pose_csv_.close();
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image.clone();

            const auto& info = camera_info_;
            const bool calibrated = info && info->header.frame_id == msg->header.frame_id && info->width == msg->width && info->height == msg->height &&
                info->k[0] > 0.0 && info->k[4] > 0.0 && std::all_of(info->k.begin(), info->k.end(), [](double value) { return std::isfinite(value); }) &&
                (info->d.empty() || (info->distortion_model == "plumb_bob" && info->d.size() == 5 && std::all_of(info->d.begin(), info->d.end(), [](double value) { return std::isfinite(value); })));

            // 整幅画面都参与检测。
            // 原实现把机关中心和 ROI 写死成固定像素（约 (639,154)、270px 半宽），
            // 云台一俯仰或靶子转到轮盘另一侧就漏检；这里改成全图 + 后面动态估计中心。
            const cv::Rect rune_roi(0, 0, frame.cols, frame.rows);

            // 机关中心：先给个回退值，找到灯条后按灯条重心更新
            int hub_x = frame.cols / 2;
            int hub_y = frame.rows / 3;

            cv::Mat blue_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
            std::vector<std::vector<cv::Point>> contours;
            
            if (!rune_roi.empty()) {
                cv::Mat blurred;
                cv::GaussianBlur(frame(rune_roi), blurred, cv::Size(5, 5), 0);
                cv::Mat hsv;
                cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
                cv::Mat roi_mask;
                cv::inRange(hsv, cv::Scalar(90, 60, 50), cv::Scalar(140, 255, 255), roi_mask);
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
                cv::morphologyEx(roi_mask, roi_mask, cv::MORPH_OPEN, kernel);
                cv::morphologyEx(roi_mask, roi_mask, cv::MORPH_CLOSE, kernel);
                cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, rune_roi.tl());
                roi_mask.copyTo(blue_mask(rune_roi));
            }

            int light_count = 0;
            std::vector<cv::Point> light_centers;

            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area < 40 || area > 2500) continue;
                cv::Rect box = cv::boundingRect(contour);
                if (box.width < 5 || box.height < 5 || box.width > 100 || box.height > 100) continue;
                double aspect = static_cast<double>(box.width) / box.height;
                if (aspect < 0.2 || aspect > 5.0) continue;
                cv::Moments m = cv::moments(contour);
                if (m.m00 <= 1e-5) continue;

                int cx = static_cast<int>(m.m10 / m.m00);
                int cy = static_cast<int>(m.m01 / m.m00);
                light_centers.emplace_back(cx, cy);
                light_count++;

                cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, cv::Point(cx, cy), 4, cv::Scalar(0, 0, 255), -1);
            }

            // 机关中心：用云台角 + 已知机关世界坐标投影到图像。
            // （原实现写死 (639,154)，云台一俯仰就偏；蓝方：发射架 y=-2.07，机关中心 (0,-0.116,0.928)）
            bool hub_ok = false;
            if (calibrated && feedback_) {
                const double yw = feedback_->yaw;
                const double pt = feedback_->pitch;
                const double cyw = std::cos(yw), syw = std::sin(yw);
                const double cpt = std::cos(pt), spt = std::sin(pt);
                // R = Rz(yaw)·Rx(pitch)
                const double fwd[3] = {-syw * cpt, cyw * cpt, spt};  // R·(0,1,0)
                const double upv[3] = {syw * spt, -cyw * spt, cpt};  // R·(0,0,1)
                double xo[3] = {fwd[1] * upv[2] - fwd[2] * upv[1],
                                fwd[2] * upv[0] - fwd[0] * upv[2],
                                fwd[0] * upv[1] - fwd[1] * upv[0]};
                const double xn = std::sqrt(xo[0] * xo[0] + xo[1] * xo[1] + xo[2] * xo[2]);
                if (xn > 1e-9) {
                    xo[0] /= xn; xo[1] /= xn; xo[2] /= xn;
                    const double yo[3] = {fwd[1] * xo[2] - fwd[2] * xo[1],
                                          fwd[2] * xo[0] - fwd[0] * xo[2],
                                          fwd[0] * xo[1] - fwd[1] * xo[0]};
                    const double camx = 0.036 * fwd[0] + 0.075 * upv[0];
                    const double camy = -2.07 + 0.036 * fwd[1] + 0.075 * upv[1];
                    const double camz = 0.495 + 0.036 * fwd[2] + 0.075 * upv[2];
                    const double dxw = 0.0 - camx;
                    const double dyw = -0.116 - camy;
                    const double dzw = 0.928 - camz;
                    const double zo = dxw * fwd[0] + dyw * fwd[1] + dzw * fwd[2];
                    if (zo > 0.1) {
                        const double xopt = dxw * xo[0] + dyw * xo[1] + dzw * xo[2];
                        const double yopt = dxw * yo[0] + dyw * yo[1] + dzw * yo[2];
                        hub_x = cvRound(info->k[2] + info->k[0] * xopt / zo);
                        hub_y = cvRound(info->k[5] + info->k[4] * yopt / zo);
                        hub_ok = true;
                    }
                }
            }
            // 投影不可用（无相机信息/无云台反馈）时退回灯条重心
            if (!hub_ok && static_cast<int>(light_centers.size()) >= 4) {
                cv::Point2d c(0.0, 0.0);
                for (const auto& p : light_centers) c += cv::Point2d(p);
                c *= 1.0 / static_cast<double>(light_centers.size());
                hub_x = cvRound(c.x);
                hub_y = cvRound(c.y);
            }

            // 机关中心不再画出来（调试用，正式画面保持干净）

            const cv::Point2d hub(hub_x, hub_y);
            std::vector<LightPair> radial_pairs;
            std::vector<LightPair> tangential_pairs;
            for (std::size_t i = 0; i < light_centers.size(); ++i) {
                for (std::size_t j = i + 1; j < light_centers.size(); ++j) {
                    const cv::Point2d first(light_centers[i]);
                    const cv::Point2d second(light_centers[j]);
                    const cv::Point2d center = (first + second) * 0.5;
                    const cv::Point2d radial = center - hub;
                    const cv::Point2d offset = second - first;
                    const double radius = cv::norm(radial);
                    const double separation = cv::norm(offset);
                    if (radius < 1.0 || separation < 0.09 * radius || separation > 0.33 * radius) continue;

                    const double alignment = std::abs(offset.dot(radial)) / (separation * radius);
                    if (alignment > 0.90)
                        radial_pairs.push_back(first.dot(radial) < second.dot(radial) ? LightPair{center, separation, first, second} : LightPair{center, separation, second, first});
                    else if (alignment < 0.30)
                        tangential_pairs.push_back({center, separation, first, second});
                }
            }

            std::vector<cv::Point> pending_centers;
            const LightPair* pending_radial = nullptr;
            const LightPair* pending_tangential = nullptr;

            for (const auto& pair : tangential_pairs) {
                const auto radial = std::find_if(radial_pairs.begin(), radial_pairs.end(), [&pair](const LightPair& candidate) {
                    return cv::norm(candidate.center - pair.center) < 0.35 * pair.separation;
                });

                const bool pending = radial != radial_pairs.end();
                const cv::Point2d center = pending ? (pair.center + radial->center) * 0.5 : pair.center;
                const cv::Point pixel(cvRound(center.x), cvRound(center.y));
                const cv::Scalar color = pending ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);

                // 大号瞄准框 & 十字准星
                const int box_size = cvRound(pair.separation * 2.5); 
                cv::Rect target_rect(pixel.x - box_size / 2, pixel.y - box_size / 2, box_size, box_size);
                cv::rectangle(frame, target_rect, color, 2);
                
                const int corner_len = box_size / 4;
                // 左上
                cv::line(frame, cv::Point(target_rect.x, target_rect.y), cv::Point(target_rect.x + corner_len, target_rect.y), color, 3);
                cv::line(frame, cv::Point(target_rect.x, target_rect.y), cv::Point(target_rect.x, target_rect.y + corner_len), color, 3);
                // 右上
                cv::line(frame, cv::Point(target_rect.x + target_rect.width, target_rect.y), cv::Point(target_rect.x + target_rect.width - corner_len, target_rect.y), color, 3);
                cv::line(frame, cv::Point(target_rect.x + target_rect.width, target_rect.y), cv::Point(target_rect.x + target_rect.width, target_rect.y + corner_len), color, 3);
                // 左下
                cv::line(frame, cv::Point(target_rect.x, target_rect.y + target_rect.height), cv::Point(target_rect.x + corner_len, target_rect.y + target_rect.height), color, 3);
                cv::line(frame, cv::Point(target_rect.x, target_rect.y + target_rect.height), cv::Point(target_rect.x, target_rect.y + target_rect.height - corner_len), color, 3);
                // 右下
                cv::line(frame, cv::Point(target_rect.x + target_rect.width, target_rect.y + target_rect.height), cv::Point(target_rect.x + target_rect.width - corner_len, target_rect.y + target_rect.height), color, 3);
                cv::line(frame, cv::Point(target_rect.x + target_rect.width, target_rect.y + target_rect.height), cv::Point(target_rect.x + target_rect.width, target_rect.y + target_rect.height - corner_len), color, 3);

                cv::drawMarker(frame, pixel, color, cv::MARKER_CROSS, 20, 2);

                cv::putText(frame, pending ? "Pending" : "Active", target_rect.tl() + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                
                if (pending) {
                    pending_centers.push_back(pixel);
                    pending_radial = &*radial;
                    pending_tangential = &pair;
                }
            }

            // 发布灯态识别结果（第一部分可验证输出）：pending=待激活, active=已激活
            if (state_pub_) {
                std_msgs::msg::String state_msg;
                state_msg.data = "pending=" + std::to_string(pending_centers.size()) +
                                 " active=" + std::to_string(tangential_pairs.size() - pending_centers.size());
                state_pub_->publish(state_msg);
            }

            std::optional<PoseMeasurement> pose_measurement;
            const char* pose_status = pending_centers.empty() ? "no_pending" : "ambiguous_pending";
            
            if (pending_centers.size() == 1) {
                const cv::Point target_center = pending_centers.front();
                pose_status = "missing_camera_info";
                if (calibrated) {
                    pose_measurement = draw_pose_reprojection(frame, *pending_radial, *pending_tangential, *info);
                    pose_status = pose_measurement ? "ok" : "pnp_failed";
                }

                double dx = target_center.x - hub_x;
                double dy = target_center.y - hub_y;
                double angle_rad = std::atan2(dy, dx);
                double angle_deg = angle_rad * 180.0 / CV_PI;
                if (angle_deg < 0) angle_deg += 360.0;

                const double current_stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
                bool velocity_valid = false;

                if (has_previous_angle_) {
                    const double dt = current_stamp_sec - previous_stamp_sec_;
                    if (dt > 0.0001 && dt < 0.5) {
                        double delta_angle = angle_deg - previous_angle_deg_;
                        if (delta_angle > 180.0) delta_angle -= 360.0;
                        else if (delta_angle < -180.0) delta_angle += 360.0;

                        const double raw_velocity = delta_angle / dt;
                        if (has_velocity_) {
                            filtered_angular_velocity_deg_s_ = 0.7 * filtered_angular_velocity_deg_s_ + 0.3 * raw_velocity;
                        } else {
                            filtered_angular_velocity_deg_s_ = raw_velocity;
                            has_velocity_ = true;
                        }
                        velocity_valid = true;
                    } else {
                        has_velocity_ = false;
                        filtered_angular_velocity_deg_s_ = 0.0;
                    }
                }

                previous_angle_deg_ = angle_deg;
                previous_stamp_sec_ = current_stamp_sec;
                has_previous_angle_ = true;

                if (velocity_valid) {
                    cv::putText(frame, cv::format("Angular Velocity: %.2f deg/s", filtered_angular_velocity_deg_s_), cv::Point(20, 125), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Angle: %.2f deg | Angular velocity: %.2f deg/s", angle_deg, filtered_angular_velocity_deg_s_);
                } else {
                    cv::putText(frame, "Angular Velocity: Calculating...", cv::Point(20, 125), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
                }

                if (velocity_valid) {
                    const double prediction_time = 0.10;
                    double predicted_angle_deg = std::fmod(angle_deg + filtered_angular_velocity_deg_s_ * prediction_time, 360.0);
                    if (predicted_angle_deg < 0.0) predicted_angle_deg += 360.0;

                    const double predicted_angle_rad = predicted_angle_deg * CV_PI / 180.0;
                    const double radius = std::hypot(dx, dy);
                    const cv::Point predicted_center(
                        cvRound(hub_x + radius * std::cos(predicted_angle_rad)),
                        cvRound(hub_y + radius * std::sin(predicted_angle_rad)));

                    cv::circle(frame, predicted_center, 10, cv::Scalar(0, 165, 255), 2);
                    cv::arrowedLine(frame, target_center, predicted_center, cv::Scalar(0, 165, 255), 2);
                    cv::putText(frame, cv::format("Predicted Angle: %.2f deg", predicted_angle_deg), cv::Point(20, 155), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
                }

                cv::circle(frame, target_center, 8, cv::Scalar(255, 0, 255), -1);
                cv::putText(frame, "Target Center: (" + std::to_string(target_center.x) + ", " + std::to_string(target_center.y) + ")", cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
                cv::putText(frame, cv::format("Rotation Angle: %.2f deg", angle_deg), cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target angle: %.2f deg", angle_deg);

            } else {
                has_previous_angle_ = false;
                has_velocity_ = false;
                cv::putText(frame, "Waiting for one pending plate", cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            cv::putText(frame, "Light bars: " + std::to_string(light_count), cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

            const std::optional<cv::Point> diagnostic_target = pending_centers.size() == 1 ? std::optional<cv::Point>(pending_centers.front()) : std::nullopt;
            show_diagnostics(frame, *msg, diagnostic_target, pose_measurement, pose_status);
            
            if (pose_measurement && !msg->header.frame_id.empty())
                publish_pose(*msg, *pose_measurement);
            write_pose_row(*msg, pose_measurement, pose_status);

            cv::imshow("Energy Rune Detection", frame);
            cv::imshow("Blue Mask", blue_mask);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
        catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    // ===== 成员变量声明 =====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<robomini_interfaces::msg::GimbalToVision>::SharedPtr feedback_sub_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;

    std::ofstream pose_csv_;
    std::ofstream diagnostic_csv_;
    std::optional<Feedback> feedback_;
    
    // 这里就是报错缺失的变量，现在补齐了
    double feedback_timeout_ms_{250.0};
    bool prefer_frontal_pose_{true};

    bool has_previous_angle_{false};
    bool has_velocity_{false};
    double previous_angle_deg_{0.0};
    double previous_stamp_sec_{0.0};
    double filtered_angular_velocity_deg_s_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LightDetector>();
    rclcpp::spin(node);
    node.reset();
    cv::destroyAllWindows();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}