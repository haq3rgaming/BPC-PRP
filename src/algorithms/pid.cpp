#include "algorithms/pid.hpp"
#include <algorithm>

namespace nodes {
    PID::PID(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        prev_error_ = 0.0;
        integral_ = 0.0;
        max_integral_ = 100.0; // Prevent integral windup
    }

    double PID::update(double target, double current, double dt)
    {
        double error = target - current;

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -max_integral_, max_integral_);

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void PID::reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }
}