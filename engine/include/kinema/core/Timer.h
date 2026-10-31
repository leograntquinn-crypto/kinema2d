// =============================================================================
// 高精度计时器：用于统计每个物理阶段的耗时
// =============================================================================
#pragma once

#include <chrono>

namespace kinema {

class Timer {
public:
    Timer() : m_start(Clock::now()) {}

    void Reset() { m_start = Clock::now(); }

    /// 自构造/Reset 以来经过的毫秒数
    [[nodiscard]] double ElapsedMs() const {
        return std::chrono::duration<double, std::milli>(Clock::now() - m_start).count();
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point m_start;
};

}  // namespace kinema
