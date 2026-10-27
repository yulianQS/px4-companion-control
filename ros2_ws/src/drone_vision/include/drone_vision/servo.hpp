#pragma once
#include <algorithm>
#include <cmath>
namespace drone_vision {
// ROS FLU yaw positive is left/CCW; pixel error positive is right.
// Returns only a preview, never a MAVLink setpoint; translation remains zero.
inline double yaw_preview(double ex, double gain, double deadzone, double limit) {
    if (!std::isfinite(ex) || !std::isfinite(gain) || !std::isfinite(deadzone) ||
        !std::isfinite(limit) || gain < 0 || deadzone < 0 || limit < 0 ||
        std::abs(ex) <= deadzone) return 0.;
    const double error = std::copysign(std::abs(ex) - deadzone, ex);
    return std::clamp(-gain * error, -limit, limit);
}
}
