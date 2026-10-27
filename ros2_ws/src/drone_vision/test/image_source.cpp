// Test-only C++ image source: same ROS publisher type/QoS as production camera.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
class Source : public rclcpp::Node {
    std::vector<unsigned char> pixels_;
    std::string phase_="stop";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    Source():Node("vision_test_image_source") {
        const auto path=declare_parameter("image_path",std::string());
        std::ifstream in(path,std::ios::binary);
        pixels_.assign(std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>());
        if(pixels_.size()!=640*480*3)throw std::runtime_error("Invalid test image");
        pub_=create_publisher<sensor_msgs::msg::Image>("/camera/image_raw",rclcpp::SensorDataQoS().keep_last(1));
        phase_sub_=create_subscription<std_msgs::msg::String>("/_vision_test/phase",1,
            [this](std_msgs::msg::String::ConstSharedPtr msg){phase_=msg->data;});
        timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{
            if(phase_=="stop")return;
            sensor_msgs::msg::Image msg;msg.header.stamp=phase_=="stale"?now()-rclcpp::Duration::from_seconds(2):now();
            msg.header.frame_id="camera_optical_frame";msg.width=640;msg.height=480;msg.step=1920;msg.encoding="bgr8";
            msg.data=phase_=="blank"?std::vector<unsigned char>(640*480*3,255):pixels_;
            if(phase_=="mono" || phase_=="stale_mono") {
                msg.encoding="mono8";msg.step=640;msg.data.resize(640*480);
                for(size_t i=0;i<msg.data.size();++i)msg.data[i]=pixels_[3*i];
                if(phase_=="stale_mono")msg.header.stamp=now()-rclcpp::Duration::from_seconds(2);
            }
            pub_->publish(std::move(msg));
        });
    }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Source>());rclcpp::shutdown();}
