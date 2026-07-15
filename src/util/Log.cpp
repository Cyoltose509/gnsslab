#include "Log.h"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#ifdef _WIN32
#include <windows.h>
#endif

std::mutex Log::mutex_;
std::string Log::path_;

void Log::init(const std::string &path) {
#ifdef _WIN32
    char absPath[MAX_PATH] = {};
    if (GetFullPathNameA(path.c_str(), MAX_PATH, absPath, nullptr) > 0)
        path_ = absPath;
    else
        path_ = path;
#else
    path_ = path;
#endif
    std::ofstream f(path_, std::ios::trunc);
}

void Log::write(const Level lvl, const std::string &msg) {
    std::lock_guard lock(mutex_);
    std::ofstream f(path_, std::ios::app);
    if (!f) return;

    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm;
    localtime_s(&tm, &tt);

    f << std::put_time(&tm, "%H:%M:%S")
      << '.' << std::setfill('0') << std::setw(3) << ms.count()
      << " [" << levelStr(lvl) << "] "
      << msg << std::endl;
}

const char *Log::levelStr(const Level lvl) {
    switch (lvl) {
        case debug: return "DEBUG";
        case info:  return "INFO ";
        case warn:  return "WARN ";
        case error: return "ERROR";
    }
    return "????";
}
