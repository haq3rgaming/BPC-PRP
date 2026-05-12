#include <rclcpp/rclcpp.hpp>

#include "algorithms/perf.hpp"

namespace nodes {
    TimePerformance::TimePerformance(const char* name) : name_(name) {
        start_time_ = std::chrono::steady_clock::now();
    }

    TimePerformance::~TimePerformance() {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();
        RCLCPP_INFO(rclcpp::get_logger(name_), "Execution time: %ld ms", duration);
    }
}