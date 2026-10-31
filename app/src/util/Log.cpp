#include "util/Log.h"

#include <chrono>
#include <cstdio>

namespace app {

namespace {

std::deque<LogEntry>& Storage() {
    static std::deque<LogEntry> entries;
    return entries;
}

double Now() {
    static const auto start = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

constexpr size_t kMaxEntries = 1000;

}  // namespace

void Log::Write(LogLevel level, const char* fmt, va_list args) {
    char buffer[1024];
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);

    const char* prefix = level == LogLevel::Info ? "[info] " : level == LogLevel::Warning ? "[warn] " : "[error] ";
    std::fprintf(level == LogLevel::Error ? stderr : stdout, "%s%s\n", prefix, buffer);

    auto& entries = Storage();
    entries.push_back({level, Now(), buffer});
    if (entries.size() > kMaxEntries) entries.pop_front();
}

void Log::Info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Write(LogLevel::Info, fmt, args);
    va_end(args);
}

void Log::Warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Write(LogLevel::Warning, fmt, args);
    va_end(args);
}

void Log::Error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    Write(LogLevel::Error, fmt, args);
    va_end(args);
}

const std::deque<LogEntry>& Log::Entries() { return Storage(); }

void Log::Clear() { Storage().clear(); }

}  // namespace app
