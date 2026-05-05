#pragma once

#include <chrono>

namespace nodes {
    class Watchdog {
    public:
        using Clock = std::chrono::steady_clock;
        explicit Watchdog(std::chrono::milliseconds timeout);
        ~Watchdog() = default;
        void kick();
        bool is_expired() const;
        std::chrono::milliseconds age() const;
    private:
        std::chrono::milliseconds timeout_;
        Clock::time_point last_kick_time_;
    };
}