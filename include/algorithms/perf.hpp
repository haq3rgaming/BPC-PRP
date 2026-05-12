#pragma once

#include <chrono>

namespace nodes {
    class TimePerformance {
    public:
        TimePerformance(const char* name = "Unnamed");
        ~TimePerformance();
    private:
        const char* name_;
        std::chrono::steady_clock::time_point start_time_;
    };
}