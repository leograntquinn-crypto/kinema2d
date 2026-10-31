// =============================================================================
// 简单的日志系统：同时输出到控制台（stdout）和编辑器的 Console 面板
// =============================================================================
#pragma once

#include <cstdarg>
#include <deque>
#include <string>

namespace app {

enum class LogLevel { Info, Warning, Error };

struct LogEntry {
    LogLevel level;
    double time;  // 程序启动后的秒数
    std::string text;
};

class Log {
public:
    static void Info(const char* fmt, ...);
    static void Warn(const char* fmt, ...);
    static void Error(const char* fmt, ...);

    static const std::deque<LogEntry>& Entries();
    static void Clear();

private:
    static void Write(LogLevel level, const char* fmt, va_list args);
};

}  // namespace app
