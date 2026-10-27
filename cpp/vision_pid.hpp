#pragma once
#include <algorithm>
#include <cmath>
// Outer-loop PID. Units: normalized pixels -> rad/s or m/s; metres -> m/s.
// Output amplitude is a user setting; zero means unbounded. No automatic
// deadband, acceleration ramp, integral clamp, or time/displacement cutoff.
struct VisionPID {
    double integral = 0, previous = 0;
    bool initialized = false;
    void reset() { integral = previous = 0; initialized = false; }
    double step(double error, double dt, double kp, double ki, double kd, double speed) {
        const double derivative = initialized ? (error - previous) / dt : 0;
        integral += error * dt;
        previous = error; initialized = true;
        const double output = kp * error + ki * integral + kd * derivative;
        return speed > 0 ? std::clamp(output, -speed, speed) : output;
    }
};
