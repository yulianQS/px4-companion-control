#include "drone_vision/pose.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cstdio>
#include <stdexcept>
void require(bool value,const char* text) {if(!value)throw std::runtime_error(text);}
int main() {
    const cv::Mat matrix=(cv::Mat_<double>(3,3)<<718.5868189684702,0,341.84200734644406,
        0,718.5063998169626,187.1627746698019,0,0,1);
    const cv::Mat distortion=(cv::Mat_<double>(1,5)<<-.11458152480112373,1.1405493040669,
        -.013753471711095725,.005431134614750439,-3.978919736781686);
    const auto path=cv::tempfile(".yaml");
    {cv::FileStorage out(path,cv::FileStorage::WRITE);
     out<<"image_width"<<640<<"image_height"<<480<<"camera_matrix"<<matrix<<"distortion_coefficients"<<distortion;}
    drone_vision::CameraModel model;model.load(path);std::remove(path.c_str());
    require(model.matches(640,480) && !model.matches(1280,960),"Calibration resolution contract");
    const float half=.06;
    std::vector<cv::Point3f> object{{-half,half,0},{half,half,0},{half,-half,0},{-half,-half,0}};
    for(const auto& translation:std::vector<cv::Vec3d>{{.02,-.03,.4},{-.1,.06,1.},{.15,-.1,2.}}) {
        for(const auto& rotation:std::vector<cv::Vec3d>{{3.14159,.1,.02},{2.6,-.2,.1},{2.8,.4,-.2}}) {
            std::vector<cv::Point2f> corners;cv::projectPoints(object,rotation,translation,matrix,distortion,corners);
            const auto pose=model.estimate(corners,.12);
            require(pose.valid && cv::norm(pose.translation-translation)<.0001,"Recover optical translation with calibrated distortion");
            require(pose.rms_px<.001,"Pose reprojection residual");
            const auto scaled=model.estimate(corners,.24);
            require(cv::norm(scaled.translation-2*translation)<.0002,"Physical tag scale");
        }
    }
    std::cout<<"PASS: actual calibration, tilted square PnP, optical-axis distance, tag scale, resolution\n";
}
