#include "drone_vision/pose.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <stdexcept>
namespace drone_vision {
void CameraModel::load(const std::string& filename) {
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("Cannot open camera calibration: " + filename);
    file["image_width"] >> width_; file["image_height"] >> height_;
    file["camera_matrix"] >> matrix_; file["distortion_coefficients"] >> distortion_;
    if (width_ < 2 || height_ < 2 || matrix_.rows != 3 || matrix_.cols != 3 ||
        distortion_.total() != 5 || !cv::checkRange(matrix_) || !cv::checkRange(distortion_))
        throw std::runtime_error("Invalid pinhole camera calibration");
    matrix_.convertTo(matrix_, CV_64F); distortion_.convertTo(distortion_, CV_64F);
    if (matrix_.at<double>(0,0) <= 0 || matrix_.at<double>(1,1) <= 0)
        throw std::runtime_error("Invalid camera focal length");
}
bool CameraModel::matches(int width, int height) const {
    return !matrix_.empty() && width == width_ && height == height_;
}
TagPose CameraModel::estimate(const std::vector<cv::Point2f>& corners, double side_m) const {
    TagPose result;
    if (matrix_.empty() || corners.size() != 4 || !std::isfinite(side_m) || side_m <= 0) return result;
    // Detector order is TL, TR, BR, BL. IPPE_SQUARE requires this object order.
    const float h = side_m * .5;
    const std::vector<cv::Point3f> object{{-h,h,0},{h,h,0},{h,-h,0},{-h,-h,0}};
    cv::Vec3d rotation, translation;
    if (!cv::solvePnP(object, corners, matrix_, distortion_, rotation, translation,
                      false, cv::SOLVEPNP_IPPE_SQUARE)) return result;
    for (double v : translation.val) if (!std::isfinite(v)) return result;
    if (translation[2] <= 0) return result;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rotation, translation, matrix_, distortion_, projected);
    double squared = 0;
    for (size_t i=0; i<4; ++i) {
        const auto d = projected[i] - corners[i]; squared += d.dot(d);
    }
    result.valid = true; result.translation = translation;
    result.rms_px = std::sqrt(squared / 4);
    return result;
}
}
