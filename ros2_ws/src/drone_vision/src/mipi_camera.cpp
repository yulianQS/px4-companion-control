#include "drone_vision/mipi_camera.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <stdexcept>
namespace drone_vision {
namespace {
int call_ioctl(int fd, unsigned long request, void* arg) {
    int result;
    do { result = ioctl(fd, request, arg); } while (result < 0 && errno == EINTR);
    return result;
}
void checked(int fd, unsigned long request, void* arg) {
    if (call_ioctl(fd, request, arg) < 0) throw std::runtime_error(std::strerror(errno));
}
double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
MipiCamera::MipiCamera(const std::string& device, int width, int height) {
    try {
        fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("Open " + device + ": " + std::strerror(errno));
        int input = 0;
        checked(fd_, VIDIOC_S_INPUT, &input);
        v4l2_format fmt{}; fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = width; fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        checked(fd_, VIDIOC_S_FMT, &fmt);
        const auto& pix = fmt.fmt.pix_mp;
        width_ = pix.width; height_ = pix.height;
        stride_ = pix.plane_fmt[0].bytesperline;
        if (!stride_) stride_ = width_;
        v4l2_capability capability{};
        checked(fd_, VIDIOC_QUERYCAP, &capability);
        // Allwinner vin reports depth=12 bits/pixel as bytesperline for packed NV12,
        // i.e. width*3/2, although luma/UV rows each have width bytes. Verified against
        // this board's vin_video.c and 640x480/460800-byte QUERYBUF, as in cv_work.
        // Apply only to this exact driver/format/size contract, never to padded generic NV12.
        if (std::strncmp(reinterpret_cast<const char*>(capability.driver),"sunxi-vin",sizeof(capability.driver))==0 &&
            pix.pixelformat==V4L2_PIX_FMT_NV12 && pix.num_planes==1 &&
            stride_==size_t(width_)*3/2 && pix.plane_fmt[0].sizeimage==size_t(width_)*height_*3/2)
            stride_=width_;
        if (pix.pixelformat != V4L2_PIX_FMT_NV12 || pix.num_planes != 1 ||
            width_ <= 0 || height_ <= 0 || width_ > 4096 || height_ > 4096 ||
            width_ % 2 || height_ % 2 || stride_ < size_t(width_))
            throw std::runtime_error("Expected single-plane, even-sized NV12");
        // Working cv_work sequence: configure sensor, KEEP fd open, then start ISP.
        const auto pos = device.rfind("video");
        if (pos == std::string::npos) throw std::runtime_error("Device must name /dev/videoN");
        const int video_id = std::stoi(device.substr(pos + 5));
        isp_ = CreateAWIspApi();
        if (!isp_ || isp_->ispApiInit() < 0) throw std::runtime_error("ISP init failed");
        isp_initialized_ = true;
        isp_id_ = isp_->ispGetIspId(video_id);
        if (isp_id_ < 0 || isp_->ispStart(isp_id_) < 0) throw std::runtime_error("ISP start failed");
        isp_started_ = true;
        v4l2_requestbuffers req{}; req.count = 3; req.type = fmt.type; req.memory = V4L2_MEMORY_MMAP;
        checked(fd_, VIDIOC_REQBUFS, &req);
        if (req.count < 2 || req.count > 32) throw std::runtime_error("Invalid buffer count");
        for (unsigned i=0; i<req.count; ++i) {
            v4l2_plane plane{};
            v4l2_buffer buf{};buf.type=fmt.type;buf.memory=V4L2_MEMORY_MMAP;buf.index=i;
            buf.length=1;buf.m.planes=&plane;
            checked(fd_,VIDIOC_QUERYBUF,&buf);
            if (plane.length < stride_ * height_ * 3 / 2) throw std::runtime_error("Short NV12 buffer: width="+std::to_string(width_)+" height="+std::to_string(height_)+" stride="+std::to_string(stride_)+" length="+std::to_string(plane.length));
            void* address=mmap(nullptr,plane.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,plane.m.mem_offset);
            if(address==MAP_FAILED) throw std::runtime_error("Camera mmap failed");
            maps_.push_back({address,plane.length});
            checked(fd_,VIDIOC_QBUF,&buf);
        }
        int type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;checked(fd_,VIDIOC_STREAMON,&type);streaming_=true;
    } catch (...) { close(); throw; }
}
MipiCamera::~MipiCamera() { close(); }
void MipiCamera::configure_exposure(const std::string& device, int microseconds) {
    const int fd=::open(device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(fd<0)throw std::runtime_error("Open ISP " + device + ": " + std::strerror(errno));
    try {
        // These units are specific to sunxi_isp (not standard V4L2's 100 us units).
        auto set=[&](unsigned id,int value) {
            v4l2_queryctrl query{};query.id=id;checked(fd,VIDIOC_QUERYCTRL,&query);
            if(query.flags & (V4L2_CTRL_FLAG_DISABLED|V4L2_CTRL_FLAG_READ_ONLY) ||
               value<query.minimum || value>query.maximum)
                throw std::runtime_error("Unsupported ISP exposure setting");
            v4l2_control control{};control.id=id;control.value=value;
            checked(fd,VIDIOC_S_CTRL,&control);
            checked(fd,VIDIOC_G_CTRL,&control);
            if(control.value!=value)throw std::runtime_error("ISP exposure setting readback mismatch");
        };
        // Fixed shutter, automatic gain. Do not repeatedly fight AE in the frame loop.
        set(V4L2_CID_EXPOSURE_AUTO,microseconds ? V4L2_EXPOSURE_SHUTTER_PRIORITY : V4L2_EXPOSURE_AUTO);
        if(microseconds)set(V4L2_CID_EXPOSURE_ABSOLUTE,microseconds);
    } catch(...) {::close(fd);throw;}
    ::close(fd);
}
void MipiCamera::close() noexcept {
    if(streaming_) {int type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;call_ioctl(fd_,VIDIOC_STREAMOFF,&type);streaming_=false;}
    if(isp_) {
        if(isp_started_) {isp_->ispStop(isp_id_);isp_->ispWaitToExit(isp_id_);isp_started_=false;}
        if(isp_initialized_) {isp_->ispApiUnInit();isp_initialized_=false;}
        DestroyAWIspApi(isp_);isp_=nullptr;
    }
    for(auto& map:maps_) munmap(map.address,map.length);
    maps_.clear();
    if(fd_>=0) {::close(fd_);fd_=-1;}
}
bool MipiCamera::read(cv::Mat& bgr,const std::atomic<bool>& stop,double timeout,double max_age,double& age,bool grayscale) {
    const double deadline=monotonic()+timeout;
    while(!stop) {
        if(monotonic()>deadline) throw std::runtime_error("Camera frame timeout");
        pollfd p{fd_,POLLIN,0};const int ready=poll(&p,1,100);
        if(ready<0 && errno==EINTR) continue;
        if(ready<0 || (p.revents&(POLLERR|POLLHUP|POLLNVAL))) throw std::runtime_error("Camera disconnected/poll error");
        if(!ready) continue;
        v4l2_plane plane{};v4l2_buffer buf{};buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory=V4L2_MEMORY_MMAP;buf.length=1;buf.m.planes=&plane;
        if(call_ioctl(fd_,VIDIOC_DQBUF,&buf)<0) {if(errno==EAGAIN)continue;throw std::runtime_error("Camera DQBUF failed");}
        try {
            const size_t needed=stride_*height_*3/2;
            if(buf.index>=maps_.size() || (buf.flags&V4L2_BUF_FLAG_ERROR) ||
               plane.bytesused<plane.data_offset || plane.bytesused-plane.data_offset<needed ||
               plane.data_offset>maps_[buf.index].length || needed>maps_[buf.index].length-plane.data_offset)
                throw std::runtime_error("Invalid/incomplete NV12 measurement");
            age=0;
            if((buf.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)==V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
                age=monotonic()-(double(buf.timestamp.tv_sec)+buf.timestamp.tv_usec*1e-6);
                if(age<-.05 || age>max_age) {checked(fd_,VIDIOC_QBUF,&buf);continue;}
            }
            cv::Mat nv12(height_*3/2,width_,CV_8UC1,
                static_cast<unsigned char*>(maps_[buf.index].address)+plane.data_offset,stride_);
            // Copy luma before requeueing the driver buffer; no borrowed mmap storage escapes.
            if(grayscale)nv12.rowRange(0,height_).copyTo(bgr);
            else cv::cvtColor(nv12,bgr,cv::COLOR_YUV2BGR_NV12);
        } catch (...) {call_ioctl(fd_,VIDIOC_QBUF,&buf);throw;}
        checked(fd_,VIDIOC_QBUF,&buf);
        return true;
    }
    return false;
}
}
