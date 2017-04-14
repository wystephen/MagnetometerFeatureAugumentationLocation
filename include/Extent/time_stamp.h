#pragma once

#include <chrono>

namespace TimeStamp {
    long double now() {
        auto tt = std::chrono::system_clock::now();
        auto t_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tt.time_since_epoch());

        double time_now(double(t_nanosec.count()) * 1e-9);
        return time_now;
    }
}