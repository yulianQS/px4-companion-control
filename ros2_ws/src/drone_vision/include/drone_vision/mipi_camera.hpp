#pragma once
#include <opencv2/core.hpp>
#include <atomic>
#include <string>
#include <vector>
#include <AWIspApi.h>
namespace drone_vision {
// Owns fd, mmap buffers and ISP; safe cleanup after partial initialization.
class MipiCamera {
    struct Mapping { void* address; size_t length; };
    int fd_ = -1, isp_id_ = -1;
    bool streaming_ = false, isp_initialized_ = false, isp_started_ = false;
    AWIspApi* isp_ = nullptr;
    std::vector<Mapping> maps_;
    int width_ = 0, height_ = 0;
    size_t stride_ = 0;
public:
    MipiCamera(const std::string& device, int width, int height);
    ~MipiCamera();
    MipiCamera(const MipiCamera&) = delete;
    MipiCamera& operator=(const MipiCamera&) = delete;
    bool read(cv::Mat& bgr, const std::atomic<bool>& stop, double timeout, double max_age,
              double& buffer_age_seconds, bool grayscale=false);
    // Allwinner ISP control uses microseconds; 0 restores automatic exposure.
    void configure_exposure(const std::string& isp_device, int exposure_time_us);
    void close() noexcept;
};
}
