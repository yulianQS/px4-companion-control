#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace drone_vision {
struct TagPose {
    bool valid = false;
    cv::Vec3d translation{};
    double rms_px = 0;
};
class CameraModel {
    cv::Mat matrix_, distortion_;
    int width_ = 0, height_ = 0;
public:
    void load(const std::string& filename);
    bool matches(int width, int height) const;
    const cv::Mat& matrix() const { return matrix_; }
    const cv::Mat& distortion() const { return distortion_; }
    TagPose estimate(const std::vector<cv::Point2f>& corners, double side_m) const;
};
}
