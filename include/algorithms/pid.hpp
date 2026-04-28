#pragma once

namespace nodes {
    class PID {
    public:
        PID(double kp, double ki, double kd);
        ~PID() = default;

        double update(double target, double current, double dt);
        void reset();

    private:
        double kp_, ki_, kd_;
        double prev_error_;
        double integral_;
        double max_integral_;
    };
}