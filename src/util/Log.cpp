#include "Log.h"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#ifdef _WIN32
#include <windows.h>
#endif
#include "Const.h"

namespace {
    // 这两个静态量故意 new 出来不析构（退出时泄漏几十字节）：
    // 下载/目录扫描线程是 detach 的，可能在 main 返回之后才落最后一行日志；
    // 若它们已随静态析构销毁，就是 UB——崩在退出瞬间且难以复现。
    std::mutex &logMutex() { static auto *m = new std::mutex(); return *m; }
    std::string &logPath() { static auto *p = new std::string(); return *p; }
}

void Log::init(const std::string &path) {
#ifdef _WIN32
    char absPath[MAX_PATH] = {};
    if (GetFullPathNameA(path.c_str(), MAX_PATH, absPath, nullptr) > 0)
        logPath() = absPath;
    else
        logPath() = path;
#else
    logPath() = path;
#endif
    std::ofstream f(path, std::ios::trunc);
}

void Log::write(const Level lvl, const std::string &msg) {
    std::lock_guard lock(logMutex());
    std::ofstream f(logPath(), std::ios::app);
    if (!f) return;

    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % MS_PER_SEC;

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
        case info:  return "INFO";
        case warn:  return "WARN";
        case error: return "ERROR";
    }
    return "????";
}
