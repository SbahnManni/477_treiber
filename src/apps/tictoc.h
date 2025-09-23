#pragma once
#include <chrono>
#include <iostream>

inline void tic();
inline void toc();

namespace {
    using clock = std::chrono::steady_clock;
    clock::time_point __tic_time;
}

inline void tic() {
    __tic_time = clock::now();
}

inline void toc() {
    auto dt = std::chrono::duration<double, std::milli>(clock::now() - __tic_time).count();
    std::cout << "Loop time: " << dt << " ms (~" << (dt > 0.0 ? 1000.0 / dt : 0.0) << " Hz)\n";
}
