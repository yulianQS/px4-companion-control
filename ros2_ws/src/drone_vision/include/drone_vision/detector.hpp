#pragma once
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
namespace drone_vision {
struct Detection {
    bool valid = false;
    cv::Point2f center{};
    std::vector<cv::Point2f> corners;
    double area = 0;
};
class Detector {
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    double denoise_sigma_=0;
public:
    Detector();
    void configure_threshold(int minimum, int maximum, int step);
    void configure_denoise(double sigma);
    Detection detect(const cv::Mat& bgr, int target_id, cv::Mat& annotated, bool annotate=true) const;
};
}
