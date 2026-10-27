#include "drone_vision/detector.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
namespace drone_vision {
Detector::Detector() : dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)),
    parameters_(cv::aruco::DetectorParameters::create()) {
    parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}
void Detector::configure_threshold(int minimum,int maximum,int step) {
    if(minimum<3 || maximum<minimum || maximum>255 || step<1)
        throw std::runtime_error("Invalid adaptive threshold windows");
    parameters_->adaptiveThreshWinSizeMin=minimum;
    parameters_->adaptiveThreshWinSizeMax=maximum;
    parameters_->adaptiveThreshWinSizeStep=step;
}
void Detector::configure_denoise(double sigma) {
    if(!std::isfinite(sigma) || sigma<0 || sigma>3)throw std::runtime_error("Invalid denoise sigma");
    denoise_sigma_=sigma;
    // Locate candidates on filtered pixels, refine corners on the original measurement.
    parameters_->cornerRefinementMethod=sigma>0?cv::aruco::CORNER_REFINE_NONE:cv::aruco::CORNER_REFINE_SUBPIX;
}
Detection Detector::detect(const cv::Mat& bgr, int target_id, cv::Mat& annotated, bool annotate) const {
    if (bgr.empty() || (bgr.type() != CV_8UC3 && bgr.type() != CV_8UC1)) throw std::runtime_error("Expected BGR8 or MONO8");
    cv::Mat gray;
    if(bgr.type()==CV_8UC1)gray=bgr;
    else cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat filtered;
    if(denoise_sigma_>0)cv::GaussianBlur(gray,filtered,cv::Size(3,3),denoise_sigma_);
    else filtered=gray;
    cv::aruco::detectMarkers(filtered, dictionary_, corners, ids, parameters_);
    if(denoise_sigma_>0)for(auto& points:corners)
        cv::cornerSubPix(gray,points,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.1));
    if (annotate) {
        if(bgr.type()==CV_8UC1)cv::cvtColor(bgr,annotated,cv::COLOR_GRAY2BGR);
        else annotated = bgr.clone();
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(annotated, corners, ids);
    } else annotated.release();
    Detection result;
    int matches = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != target_id) continue;
        ++matches;
        if (corners[i].size() != 4) continue;
        bool finite = true;
        cv::Point2f center{};
        for (const auto& p : corners[i]) {
            finite = finite && std::isfinite(p.x) && std::isfinite(p.y) &&
                     p.x >= 0 && p.y >= 0 && p.x < bgr.cols && p.y < bgr.rows;
            center += p * .25f;
        }
        const double area = std::abs(cv::contourArea(corners[i]));
        if (!finite || area < 16 || !cv::isContourConvex(corners[i])) continue;
        result = {true, center, corners[i], area};
    }
    // Duplicate target IDs are ambiguous; never arbitrarily switch between them.
    if (matches != 1) result = Detection{};
    if (annotate && result.valid) cv::circle(annotated, result.center, 5, cv::Scalar(0,255,0), -1);
    return result;
}
}
