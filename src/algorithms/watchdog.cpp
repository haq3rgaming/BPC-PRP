#include "algorithms/watchdog.hpp"

namespace nodes {
    Watchdog::Watchdog(std::chrono::milliseconds timeout)
        : timeout_(timeout), last_kick_time_(Clock::now()) {}

    void Watchdog::kick() {
        last_kick_time_ = Clock::now();
    }

    bool Watchdog::is_expired() const {
        return age() >= timeout_;
    }

    std::chrono::milliseconds Watchdog::age() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_kick_time_);
    }
}