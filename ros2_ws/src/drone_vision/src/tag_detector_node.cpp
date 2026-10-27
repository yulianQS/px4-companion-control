#include "drone_vision/detector.hpp"
#include "drone_vision/pose.hpp"
#include "drone_vision/msg/target2_d.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <sys/prctl.h>
namespace drone_vision {
class TagNode : public rclcpp::Node {
    Detector detector_;
    CameraModel model_;
    std::string calibration_file_;
    double tag_size_;
    int target_id_, jpeg_quality_;
    double max_age_;
    double preview_fps_;
    std::mutex preview_mutex_;
    std::condition_variable preview_wake_;
    std::thread preview_worker_;
    bool stopping_=false;
    sensor_msgs::msg::Image::ConstSharedPtr pending_preview_;
    Detection pending_detection_;
    std::chrono::steady_clock::time_point last_image_{};
    std::chrono::steady_clock::time_point next_preview_{};
    std::chrono::steady_clock::time_point rate_started_=std::chrono::steady_clock::now();
    size_t processed_=0, found_=0;
    double processing_ms_=0;
    rclcpp::Publisher<msg::Target2D>::SharedPtr targets_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_;
    rclcpp::TimerBase::SharedPtr timer_;
    void encode_preview() {
        prctl(PR_SET_NAME,"preview-jpeg",0,0,0);
        auto next=std::chrono::steady_clock::now();
        while(true) {
            cv::Mat image;
            sensor_msgs::msg::Image::ConstSharedPtr source;
            Detection detection;
            sensor_msgs::msg::CompressedImage preview;
            {
                std::unique_lock<std::mutex> lock(preview_mutex_);
                preview_wake_.wait(lock,[this]{return stopping_ || bool(pending_preview_);});
                preview_wake_.wait_until(lock,next,[this]{return stopping_;});
                if(stopping_)return;
                source=std::move(pending_preview_);detection=std::move(pending_detection_);
                preview.header=source->header;
            }
            next=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1/preview_fps_));
            if((now()-rclcpp::Time(preview.header.stamp)).seconds()>max_age_)continue;
            preview.format="bgr8; jpeg compressed bgr8";
            try {
                cv::Mat view(source->height,source->width,source->encoding=="mono8"?CV_8UC1:CV_8UC3,
                             const_cast<uint8_t*>(source->data.data()),source->step);
                if(source->encoding=="mono8")cv::cvtColor(view,image,cv::COLOR_GRAY2BGR);
                else image=view.clone();
                if(detection.valid) {
                    cv::aruco::drawDetectedMarkers(image,std::vector<std::vector<cv::Point2f>>{detection.corners},std::vector<int>{target_id_});
                    cv::circle(image,detection.center,5,cv::Scalar(0,255,0),-1);
                }
                if(cv::imencode(".jpg",image,preview.data,{cv::IMWRITE_JPEG_QUALITY,jpeg_quality_}))
                    annotated_->publish(preview);
            } catch(const std::exception& e) {
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Preview rejected: %s",e.what());
            }
        }
    }
    void invalid(const std::string& reason) {
        msg::Target2D target;target.header.stamp=now();target.header.frame_id="camera_optical_frame";
        target.tag_id=target_id_;target.reason=reason;targets_->publish(target);
    }
    void image(sensor_msgs::msg::Image::ConstSharedPtr source) {
        const auto& msg=*source;
        try {
            const bool mono=msg.encoding=="mono8";
            const double age=(now()-rclcpp::Time(msg.header.stamp)).seconds();
            if(!std::isfinite(age) || age<-.05 || age>max_age_ || (!mono && msg.encoding!="bgr8") ||
               msg.is_bigendian || msg.width<2 || msg.height<2 || msg.width>4096 || msg.height>4096 ||
               msg.step<msg.width*(mono?1:3) || uint64_t(msg.step)*msg.height>msg.data.size())
                throw std::runtime_error(mono?"Invalid/stale MONO8 image":"Invalid/stale BGR8 image");
            // Read-only view. The preview worker retains shared ownership of its source message.
            cv::Mat view(msg.height,msg.width,mono?CV_8UC1:CV_8UC3,const_cast<uint8_t*>(msg.data.data()),msg.step),drawn;
            const auto tick=std::chrono::steady_clock::now();
            const bool preview_due=tick>=next_preview_;
            const auto result=detector_.detect(view,target_id_,drawn,false);
            ++processed_;found_+=result.valid;
            processing_ms_+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
            if((now()-rclcpp::Time(msg.header.stamp)).seconds()>max_age_)
                throw std::runtime_error("Detection exceeded freshness limit");
            last_image_=std::chrono::steady_clock::now();
            drone_vision::msg::Target2D target;target.header=msg.header;target.tag_id=target_id_;
            target.image_width=msg.width;target.image_height=msg.height;target.valid=result.valid;
            target.calibrated=model_.matches(msg.width,msg.height);
            target.tag_size_m=tag_size_;
            target.reason=result.valid?"target_detected":"target_not_found_or_ambiguous";
            if(result.valid) {
                target.center_u=result.center.x;target.center_v=result.center.y;target.area_px2=result.area;
                for(size_t i=0;i<4;++i){target.corners_uv[2*i]=result.corners[i].x;target.corners_uv[2*i+1]=result.corners[i].y;}
                if(target.calibrated) {
                    const auto pose=model_.estimate(result.corners,tag_size_);
                    target.pose_valid=pose.valid;target.pose_rms_px=pose.rms_px;
                    for(size_t i=0;i<3;++i)target.translation_m[i]=pose.translation[i];
                }
            }
            targets_->publish(target);
            if(preview_due) {
                const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1/preview_fps_));
                next_preview_+=period;
                if(next_preview_<=tick)next_preview_=tick+period;
                std::lock_guard<std::mutex> lock(preview_mutex_);
                pending_preview_=std::move(source);pending_detection_=result;
                preview_wake_.notify_one(); // Latest only; annotation follows preview rate, not detector rate.
            }
        } catch(const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Image rejected: %s",e.what());
            invalid(e.what());
        }
    }
    // Startup-only settings: edit YAML and relaunch; reject ineffective runtime writes.
    template<typename T> auto configured(const std::string& name, const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=true;
        return this->declare_parameter(name,value,descriptor);
    }
public:
    TagNode():Node("tag_detector_node") {
        target_id_=configured("target_id",0);max_age_=configured("max_image_age_seconds",.5);
        jpeg_quality_=configured("jpeg_quality",75);
        preview_fps_=configured("preview_fps",10.);
        const int threads=configured("opencv_threads",1);
        detector_.configure_denoise(configured("denoise_sigma",0.));
        detector_.configure_threshold(configured("adaptive_thresh_win_min",3),
                                      configured("adaptive_thresh_win_max",23),
                                      configured("adaptive_thresh_win_step",10));
        tag_size_=configured("tag_size_m",.12);
        calibration_file_=configured("calibration_file",std::string());
        if(threads<1 || !std::isfinite(preview_fps_) || preview_fps_<=0 ||
           target_id_<0 || target_id_>=587 || !std::isfinite(max_age_) || max_age_<=0 || max_age_>2 ||
           jpeg_quality_<1 || jpeg_quality_>100 || !std::isfinite(tag_size_) || tag_size_<=0)
            throw std::runtime_error("Invalid detector parameters");
        if(!calibration_file_.empty())model_.load(calibration_file_);
        RCLCPP_INFO(get_logger(),"Tag side %.3fm; calibration: %s",tag_size_,calibration_file_.c_str());
        cv::setNumThreads(threads);
        targets_=create_publisher<msg::Target2D>("/vision/target_2d",rclcpp::SensorDataQoS().keep_last(1));
        annotated_=create_publisher<sensor_msgs::msg::CompressedImage>("/vision/annotated/compressed",rclcpp::SensorDataQoS().keep_last(1));
        input_=create_subscription<sensor_msgs::msg::Image>("/camera/image_raw",rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg){image(std::move(msg));});
        timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
            const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-rate_started_).count();
            if(seconds>=5) {
                RCLCPP_INFO(get_logger(),"Vision rate: processed=%.2f Hz, found=%.2f Hz, mean_detect=%.2f ms (%zu frames)",
                            processed_/seconds,found_/seconds,processed_?processing_ms_/processed_:0.,processed_);
                processed_=found_=0;processing_ms_=0;rate_started_=std::chrono::steady_clock::now();
            }
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-last_image_).count()>max_age_)
                invalid("camera_missing_or_stale");
        });
        preview_worker_=std::thread([this]{encode_preview();});
    }
    ~TagNode() override {
        {std::lock_guard<std::mutex> lock(preview_mutex_);stopping_=true;}
        preview_wake_.notify_one();
        if(preview_worker_.joinable())preview_worker_.join();
    }
};
}
int main(int argc,char**argv) {
    prctl(PR_SET_NAME,"tag-detect",0,0,0);
    rclcpp::init(argc,argv);
    try {rclcpp::spin(std::make_shared<drone_vision::TagNode>());}
    catch(const std::exception& e){std::fprintf(stderr,"tag_detector_node: %s\n",e.what());rclcpp::shutdown();return 1;}
    rclcpp::shutdown();return 0;
}
