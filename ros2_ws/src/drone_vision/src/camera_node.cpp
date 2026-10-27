#include "drone_vision/mipi_camera.hpp"
#include "drone_vision/pose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <sys/prctl.h>
#include <cmath>
#include <opencv2/imgproc.hpp>
namespace drone_vision {
class CameraNode : public rclcpp::Node {
    CameraModel model_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    void status(const std::string& text) {std_msgs::msg::String msg;msg.data=text;status_->publish(msg);}
    void run() {
        prctl(PR_SET_NAME,"camera-capture",0,0,0);
        const auto device=get_parameter("device").as_string(), frame=get_parameter("frame_id").as_string();
        const int w=get_parameter("width").as_int(), h=get_parameter("height").as_int();
        const int rotation=get_parameter("image_rotation_deg").as_int();
        const bool mirror=get_parameter("mirror_horizontal").as_bool();
        const bool grayscale=get_parameter("grayscale").as_bool();
        const double fps=get_parameter("publish_fps").as_double();
        const double retry=get_parameter("retry_seconds").as_double(), timeout=get_parameter("frame_timeout_seconds").as_double();
        const double max_age=get_parameter("max_buffer_age_seconds").as_double();
        const auto isp_device=get_parameter("isp_device").as_string();
        const int exposure=get_parameter("exposure_time_us").as_int();
        while(rclcpp::ok() && !stop_) {
            try {
                MipiCamera camera(device,w,h);
                bool exposure_configured=false;
                auto next=std::chrono::steady_clock::now();
                auto rate_started=next;
                size_t captured=0,published=0;
                cv::Mat bgr;
                while(rclcpp::ok() && !stop_) {
                    double age=0;
                    if(!camera.read(bgr,stop_,timeout,max_age,age,grayscale))break;
                    ++captured;
                    if(!exposure_configured) {
                        // First frame confirms ISP startup; controls are handled by its existing event thread.
                        camera.configure_exposure(isp_device,exposure);
                        RCLCPP_INFO(get_logger(),"ISP exposure: %s, requested %d us",
                                    exposure ? "fixed shutter / automatic gain" : "automatic",exposure);
                        exposure_configured=true;
                        continue; // This frame predates the requested exposure setting.
                    }
                    const auto tick=std::chrono::steady_clock::now();
                    if(tick<next)continue; // Drain at sensor rate; publish at configured software rate.
                    const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/fps));
                    // Accumulate deadlines: resetting to tick+period can turn a 50Hz sensor into 25Hz at a 30Hz limit.
                    next+=period;
                    if(next<=tick)next=tick+period; // Never burst old frames after a stall.
                    if(rotation==90)cv::rotate(bgr,bgr,cv::ROTATE_90_CLOCKWISE);
                    else if(rotation==180)cv::rotate(bgr,bgr,cv::ROTATE_180);
                    else if(rotation==270)cv::rotate(bgr,bgr,cv::ROTATE_90_COUNTERCLOCKWISE);
                    if(mirror)cv::flip(bgr,bgr,1);
                    sensor_msgs::msg::Image msg;
                    msg.header.stamp=now()-rclcpp::Duration::from_seconds(std::max(0.,age));
                    msg.header.frame_id=frame;msg.width=bgr.cols;msg.height=bgr.rows;
                    msg.encoding=grayscale?"mono8":"bgr8";msg.is_bigendian=false;msg.step=bgr.cols*bgr.elemSize();
                    msg.data.assign(bgr.data,bgr.data+bgr.total()*bgr.elemSize());
                    sensor_msgs::msg::CameraInfo ci;ci.header=msg.header;ci.width=msg.width;ci.height=msg.height;
                    const bool calibrated=rotation==0 && !mirror && model_.matches(msg.width,msg.height);
                    if(calibrated) {
                        ci.distortion_model="plumb_bob";
                        const auto& matrix=model_.matrix();
                        const auto distortion=model_.distortion().reshape(1,1);
                        for(size_t i=0;i<9;++i)ci.k[i]=matrix.at<double>(i/3,i%3);
                        ci.d.assign(distortion.ptr<double>(),distortion.ptr<double>()+distortion.total());
                        ci.r={1,0,0,0,1,0,0,0,1};
                        ci.p={ci.k[0],ci.k[1],ci.k[2],0,ci.k[3],ci.k[4],ci.k[5],0,ci.k[6],ci.k[7],ci.k[8],0};
                    }
                    info_->publish(ci);images_->publish(std::move(msg));
                    ++published;
                    const double seconds=std::chrono::duration<double>(tick-rate_started).count();
                    if(seconds>=5) {
                        RCLCPP_INFO(get_logger(),"Camera rate: captured=%.2f Hz, published=%.2f Hz, encoding=%s",
                                    captured/seconds,published/seconds,grayscale?"mono8":"bgr8");
                        captured=published=0;rate_started=tick;
                    }
                    status(std::string("camera_ready; ")+(calibrated?"calibrated; ":"uncalibrated; ")+std::to_string(bgr.cols)+"x"+std::to_string(bgr.rows));
                }
            } catch(const std::exception& e) {
                status(std::string("camera_error: ")+e.what());
                RCLCPP_WARN(get_logger(),"%s",e.what());
            }
            for(int i=0;i<int(retry*10) && !stop_ && rclcpp::ok();++i)std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    // Startup-only settings: edit YAML and relaunch; reject ineffective runtime writes.
    template<typename T> auto configured(const std::string& name, const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=true;
        return this->declare_parameter(name,value,descriptor);
    }
public:
    CameraNode():Node("camera_node") {
        configured("device","/dev/video0");configured("width",640);configured("height",480);
        configured("isp_device","/dev/v4l-subdev12");
        const int exposure=configured("exposure_time_us",0);
        if(exposure<0 || exposure>30000000)throw std::runtime_error("exposure_time_us must be 0 (auto) or 1..30000000");
        const int rotation=configured("image_rotation_deg",0);
        configured("mirror_horizontal",false);
        configured("grayscale",false); // Preserve BGR8 for callers without a config file.
        const auto calibration=configured("calibration_file",std::string());
        if(!calibration.empty())model_.load(calibration);
        if(rotation!=0 && rotation!=90 && rotation!=180 && rotation!=270)throw std::runtime_error("Rotation must be 0/90/180/270");
        configured("publish_fps",10.);configured("frame_id","camera_optical_frame");
        configured("retry_seconds",2.);configured("frame_timeout_seconds",2.);configured("max_buffer_age_seconds",.25);
        const double fps=get_parameter("publish_fps").as_double();
        if(!std::isfinite(fps) || fps<1 || fps>30 || get_parameter("width").as_int()<2 ||
           get_parameter("height").as_int()<2 || get_parameter("width").as_int()>4096 || get_parameter("height").as_int()>4096 ||
           !std::isfinite(get_parameter("retry_seconds").as_double()) || get_parameter("retry_seconds").as_double()<.1 ||
           get_parameter("retry_seconds").as_double()>60 || !std::isfinite(get_parameter("frame_timeout_seconds").as_double()) ||
           get_parameter("frame_timeout_seconds").as_double()<=0 || get_parameter("frame_timeout_seconds").as_double()>10 ||
           !std::isfinite(get_parameter("max_buffer_age_seconds").as_double()) ||
           get_parameter("max_buffer_age_seconds").as_double()<=0 || get_parameter("max_buffer_age_seconds").as_double()>1)
            throw std::runtime_error("Invalid camera parameters");
        cv::setNumThreads(1);
        images_=create_publisher<sensor_msgs::msg::Image>("/camera/image_raw",rclcpp::SensorDataQoS().keep_last(1));
        info_=create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info",rclcpp::SensorDataQoS().keep_last(1));
        status_=create_publisher<std_msgs::msg::String>("/camera/status",1);
        worker_=std::thread([this]{run();});
    }
    ~CameraNode() override {stop_=true;if(worker_.joinable())worker_.join();}
};
}
int main(int argc,char**argv) {
    rclcpp::init(argc,argv);
    try {rclcpp::spin(std::make_shared<drone_vision::CameraNode>());}
    catch(const std::exception& e){std::fprintf(stderr,"camera_node: %s\n",e.what());rclcpp::shutdown();return 1;}
    rclcpp::shutdown();return 0;
}
