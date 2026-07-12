#pragma once

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

/// 简易日志类（线程安全，文件输出）
/// 用法：LOG_INFO("sat count=" << n);  LOG_ERROR("parse failed: " << e.what());
class Log {
public:
    enum Level { DEBUG_L, INFO_L, WARN_L, ERROR_L };

    /// 初始化（启动时调用，清空旧日志）
    static void init(const std::string &path);

    /// 写日志（通过宏调用，不建议直接使用）
    static void write(Level lvl, const char *file, int line, const std::string &msg);

private:
    static std::mutex mutex_;
    static std::string path_;
    static const char *levelStr(Level lvl);
};

// ---- 便捷宏 ----
#define LOG_DEBUG(msg) do { \
    std::ostringstream _oss; _oss << (msg); \
    Log::write(Log::DEBUG_L, __FILE__, __LINE__, _oss.str()); \
} while(0)

#define LOG_INFO(msg) do { \
    std::ostringstream _oss; _oss << (msg); \
    Log::write(Log::INFO_L, __FILE__, __LINE__, _oss.str()); \
} while(0)

#define LOG_WARN(msg) do { \
    std::ostringstream _oss; _oss << (msg); \
    Log::write(Log::WARN_L, __FILE__, __LINE__, _oss.str()); \
} while(0)

#define LOG_ERROR(msg) do { \
    std::ostringstream _oss; _oss << (msg); \
    Log::write(Log::ERROR_L, __FILE__, __LINE__, _oss.str()); \
} while(0)

// ---- 实现 (header-only) ----
#ifdef LOG_IMPL
#include <chrono>
#include <ctime>
#include <iomanip>

std::mutex Log::mutex_;
std::string Log::path_;

inline void Log::init(const std::string &path) {
    path_ = path;
    // 启动时清空
    std::ofstream f(path_, std::ios::trunc);
}

inline void Log::write(const Level lvl, const char *file, const int line, const std::string &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream f(path_, std::ios::app);
    if (!f) return;

    // 时间戳
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm;
    localtime_s(&tm, &tt);

    f << std::put_time(&tm, "%H:%M:%S")
      << '.' << std::setfill('0') << std::setw(3) << ms.count()
      << " [" << levelStr(lvl) << "] "
      << file << ':' << line << "  "
      << msg << std::endl;
}

inline const char *Log::levelStr(Level lvl) {
    switch (lvl) {
        case DEBUG_L: return "DEBUG";
        case INFO_L:  return "INFO ";
        case WARN_L:  return "WARN ";
        case ERROR_L: return "ERROR";
    }
    return "????";
}
#endif // LOG_IMPL
