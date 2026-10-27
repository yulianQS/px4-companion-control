#include "drone_vision/msg/target2_d.hpp"
#include "drone_vision/servo.hpp"
#include "json.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
namespace drone_vision {
using Json=nlohmann::json;
namespace fs=std::filesystem;
// Latest shared-memory snapshots let existing Python UI read without a rclpy/venv dependency.
void atomic_write(const fs::path& path,const char* data,size_t size) {
    const auto tmp=path.string()+".tmp";
    {std::ofstream out(tmp,std::ios::binary|std::ios::trunc);out.write(data,size);
     if(!out)throw std::runtime_error("Cannot write vision snapshot");}
    fs::rename(tmp,path);
}
double steady_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class FollowNode : public rclcpp::Node {
    msg::Target2D target_;
    double received_=0,max_age_,gain_,limit_,deadzone_;
    bool upright_,preview_enabled_=true;
    Json preview_defaults_,preview_parameters_;
    std::string preview_revision_;
    fs::path directory_;
    std::string camera_status_="camera_starting";
    sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_image_;
    double image_received_=0;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr commands_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Subscription<msg::Target2D>::SharedPtr input_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_;
    rclcpp::TimerBase::SharedPtr timer_;
    void refresh_preview_config() {
        try {
            Json candidate=preview_defaults_;
            std::string revision;
            const auto path=directory_/"preview_config.json";
            if(fs::exists(path)) {
                std::ifstream in(path,std::ios::binary);
                char data[4097];in.read(data,sizeof(data));
                const auto size=in.gcount();
                if(!in.eof() || size<=0 || size>4096)throw std::runtime_error("Invalid preview settings size");
                const auto root=Json::parse(data,data+size);
                revision=root.at("revision").get<std::string>();
                candidate=root.at("parameters");
                if(revision.empty() || revision.size()>64 || !candidate.is_object() || candidate.size()!=6)
                    throw std::runtime_error("Invalid preview settings contract");
            }
            for(const auto* key:{"yaw_gain","max_yaw_rate","deadzone_normalized","max_target_age_seconds"})
                if(!candidate.at(key).is_number())throw std::runtime_error("Preview settings must be numeric");
            const double gain=candidate.at("yaw_gain").get<double>(),limit=candidate.at("max_yaw_rate").get<double>();
            const double deadzone=candidate.at("deadzone_normalized").get<double>(),age=candidate.at("max_target_age_seconds").get<double>();
            if(!candidate.at("preview_enabled").is_boolean() || !candidate.at("camera_upright_confirmed").is_boolean() ||
               !std::isfinite(gain) || !std::isfinite(limit) || limit<0 || limit>.5 ||
               !std::isfinite(deadzone) || deadzone<0 || deadzone>.5 || !std::isfinite(age) || age<=0 || age>2)
                throw std::runtime_error("Invalid preview settings values");
            // Commit all settings together only after validation; never alter camera or PX4 settings.
            gain_=gain;limit_=limit;deadzone_=deadzone;max_age_=age;
            upright_=candidate.at("camera_upright_confirmed").get<bool>();
            preview_enabled_=candidate.at("preview_enabled").get<bool>();
            preview_parameters_=candidate;preview_revision_=revision;
        }catch(const std::exception& e){
            preview_enabled_=false;preview_revision_.clear();
            upright_=false;preview_parameters_["camera_upright_confirmed"]=false;
            preview_parameters_["preview_enabled"]=false;
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Preview disabled: %s",e.what());
        }
    }
    void tick() {
        refresh_preview_config();
        const auto stamp=now(); const double steady=steady_seconds();
        const double age=(stamp-rclcpp::Time(target_.header.stamp)).seconds();
        // Missing-camera heartbeats carry no image dimensions; they are not fresh camera frames.
        const bool recent=received_>0 && steady-received_<max_age_ && age>=-.05 && age<max_age_ &&
                          target_.image_width>=2 && target_.image_height>=2;
        bool valid=recent && target_.valid && target_.image_width>=2 && target_.image_height>=2 &&
            target_.image_width<=4096 && target_.image_height<=4096 && target_.header.frame_id=="camera_optical_frame" &&
            std::isfinite(target_.center_u) && std::isfinite(target_.center_v) &&
            target_.center_u>=0 && target_.center_v>=0 && target_.center_u<target_.image_width &&
            target_.center_v<target_.image_height && std::isfinite(target_.area_px2) && target_.area_px2>=16 &&
            target_.area_px2<=double(target_.image_width)*target_.image_height;
        for(size_t i=0;i<8;++i) valid=valid && std::isfinite(target_.corners_uv[i]) &&
            target_.corners_uv[i]>=0 && target_.corners_uv[i]<(i%2?target_.image_height:target_.image_width);
        const double ex=valid?(target_.center_u-(target_.image_width-1)*.5)/(target_.image_width*.5):0;
        const double ey=valid?(target_.center_v-(target_.image_height-1)*.5)/(target_.image_height*.5):0;
        geometry_msgs::msg::TwistStamped cmd;cmd.header.stamp=stamp;cmd.header.frame_id="base_link";
        if(valid && preview_enabled_)cmd.twist.angular.z=ex*gain_;
        commands_->publish(cmd); // Preview ONLY; no UART/TCP, arming, mode or MAVLink connection.
        const bool image_fresh=latest_image_ && steady-image_received_<max_age_ &&
            (stamp-rclcpp::Time(latest_image_->header.stamp)).seconds()>=-.05 &&
            (stamp-rclcpp::Time(latest_image_->header.stamp)).seconds()<max_age_;
        // Control validity follows timestamped detection measurements, never JPEG delivery or browser viewers.
        Json state={{"written_monotonic",steady},{"source","ros2_cpp"},{"camera_ready",recent},
            {"observation_monotonic",received_},
            {"preview_image_ready",image_fresh},
            {"target_found",valid},{"tag_id",target_.tag_id},{"width",target_.image_width},{"height",target_.image_height},
            {"center",valid?Json::array({target_.center_u,target_.center_v}):Json(nullptr)},
            {"normalized_error",valid?Json::array({ex,ey}):Json(nullptr)},
            {"measurement_age_s",received_>0?Json(age):Json(nullptr)},
            {"pose_valid",valid && target_.pose_valid},{"calibrated",target_.calibrated},{"actuation_enabled",false},
            {"translation_m",valid && target_.pose_valid?Json(target_.translation_m):Json(nullptr)},
            {"distance_m",valid && target_.pose_valid?Json(target_.translation_m[2]):Json(nullptr)},
            {"tag_size_m",target_.tag_size_m},{"pose_rms_px",target_.pose_rms_px},
            {"camera_upright_confirmed",upright_},{"preview_enabled",preview_enabled_},
            {"preview_parameters",preview_parameters_},{"preview_config_revision",preview_revision_},{"yaw_preview_rad_s",cmd.twist.angular.z},
            {"linear_preview_m_s",Json::array({0,0,0})},{"command_frame","base_link (ROS FLU)"},
            {"message",!preview_enabled_?"preview_disabled":!recent?"target_stream_stale":!valid?(target_.valid?"invalid_target_measurement":target_.reason):"yaw_preview_only"},
            {"camera_message",camera_status_}};
        const auto text=state.dump();std_msgs::msg::String msg;msg.data=text;status_->publish(msg);
        try {
            atomic_write(directory_/"status.json",text.data(),text.size());
        } catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"%s",e.what());}
    }
    // Startup-only settings: edit YAML and relaunch; reject ineffective runtime writes.
    template<typename T> auto configured(const std::string& name, const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=true;
        return this->declare_parameter(name,value,descriptor);
    }
public:
    FollowNode():Node("follow_preview_node") {
        max_age_=configured("max_target_age_seconds",.5);gain_=configured("yaw_gain",.3);
        limit_=configured("max_yaw_rate",.25);deadzone_=configured("deadzone_normalized",.05);
        upright_=configured("camera_upright_confirmed",false);
        directory_=configured("snapshot_directory","/dev/shm/drone_vision");
        if(!std::isfinite(max_age_) || max_age_<=0 || max_age_>2 || !std::isfinite(gain_) ||
           !std::isfinite(limit_) || limit_<0 || limit_>.5 || !std::isfinite(deadzone_) || deadzone_<0 || deadzone_>.5)
            throw std::runtime_error("Invalid follow preview parameters");
        if(directory_.empty() || !directory_.is_absolute())throw std::runtime_error("Snapshot directory must be absolute");
        preview_defaults_={{"yaw_gain",gain_},{"max_yaw_rate",limit_},{"deadzone_normalized",deadzone_},
            {"max_target_age_seconds",max_age_},{"camera_upright_confirmed",upright_},{"preview_enabled",true}};
        preview_parameters_=preview_defaults_;
        fs::create_directories(directory_);
        commands_=create_publisher<geometry_msgs::msg::TwistStamped>("/follow/cmd_preview",1);
        status_=create_publisher<std_msgs::msg::String>("/follow/status",1);
        input_=create_subscription<msg::Target2D>("/vision/target_2d",rclcpp::SensorDataQoS().keep_last(1),
            [this](msg::Target2D::ConstSharedPtr msg){target_=*msg;received_=steady_seconds();});
        image_=create_subscription<sensor_msgs::msg::CompressedImage>("/vision/annotated/compressed",rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){
                const double age=(now()-rclcpp::Time(msg->header.stamp)).seconds();
                if(age<-.05 || age>=max_age_ || msg->data.empty())return;
                latest_image_=msg;image_received_=steady_seconds();
                // Image delivery follows detection rate, independent of the 10Hz preview/status timer.
                try {atomic_write(directory_/"latest.jpg",reinterpret_cast<const char*>(msg->data.data()),msg->data.size());}
                catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"%s",e.what());}
            });
        camera_=create_subscription<std_msgs::msg::String>("/camera/status",1,
            [this](std_msgs::msg::String::ConstSharedPtr msg){camera_status_=msg->data;});
        timer_=create_wall_timer(std::chrono::milliseconds(33),[this]{tick();});
    }
};
}
int main(int argc,char**argv) {
    rclcpp::init(argc,argv);
    try {rclcpp::spin(std::make_shared<drone_vision::FollowNode>());}
    catch(const std::exception& e){std::fprintf(stderr,"follow_preview_node: %s\n",e.what());rclcpp::shutdown();return 1;}
    rclcpp::shutdown();return 0;
}
