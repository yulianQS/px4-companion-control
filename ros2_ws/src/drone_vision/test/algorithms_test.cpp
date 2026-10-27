#include "drone_vision/detector.hpp"
#include "drone_vision/servo.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <limits>
#include <iostream>
#include <fstream>
void require(bool value,const char* text) {if(!value)throw std::runtime_error(text);}
int main(int argc,char**argv) {
    drone_vision::Detector detector;
    cv::Mat marker;cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),0,160,marker);
    cv::Mat gray(480,640,CV_8UC1,cv::Scalar(255));marker.copyTo(gray(cv::Rect(350,140,160,160)));
    cv::Mat bgr,drawn;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
    auto result=detector.detect(bgr,0,drawn);
    require(result.valid && std::abs(result.center.x-429.5)<2 && result.area>20000,"36h11 ID0 pixels");
    auto without_preview=detector.detect(bgr,0,drawn,false);
    require(without_preview.valid && without_preview.corners==result.corners && drawn.empty(),"Skipping preview preserves detector result");
    const auto mono=detector.detect(gray,0,drawn,false);
    require(mono.valid && mono.corners==result.corners,"MONO8 preserves corners and calibration coordinates");
    const auto mono_preview=detector.detect(gray,0,drawn);
    require(mono_preview.valid && drawn.type()==CV_8UC3 && cv::norm(mono_preview.center-result.center)<1,"Grayscale detection and BGR preview");
    detector.configure_denoise(.8);
    const auto filtered=detector.detect(gray,0,drawn,false);
    require(filtered.valid && cv::norm(filtered.center-result.center)<1,"Denoise keeps original-image corner coordinates");
    detector.configure_denoise(0);
    require(!detector.detect(bgr,1,drawn).valid,"Wrong ID rejected");
    require(!detector.detect(cv::Mat(480,640,CV_8UC3,cv::Scalar(255,255,255)),0,drawn).valid,"Blank image loss");
    if(argc==2) {std::ofstream out(argv[1],std::ios::binary);
        cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);out.write(reinterpret_cast<const char*>(bgr.data),bgr.total()*bgr.elemSize());
        require(bool(out),"Write synthetic BGR frame");}
    marker.copyTo(gray(cv::Rect(40,140,160,160)));cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
    require(!detector.detect(bgr,0,drawn).valid,"Duplicate target ambiguity");
    require(drone_vision::yaw_preview(.5,.3,.05,.25)<0,"Right image -> negative ROS yaw");
    require(drone_vision::yaw_preview(-.5,.3,.05,.25)>0,"Left image -> positive ROS yaw");
    require(drone_vision::yaw_preview(.01,.3,.05,.25)==0,"Deadzone");
    require(drone_vision::yaw_preview(10,1,.05,.25)==-.25,"Saturation");
    require(drone_vision::yaw_preview(std::numeric_limits<double>::quiet_NaN(),1,.05,.25)==0,"Nonfinite");
    std::cout<<"PASS: AprilTag pixels/ID/loss/ambiguity, yaw sign/deadzone/limit/nonfinite\n";
}
