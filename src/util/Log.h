#pragma once


#include <mutex>
#include <sstream>
#include <string>


class Log {
public:
    enum Level { debug, info, warn, error };

    static void init(const std::string &path);
    static void write(Level lvl,  const std::string &msg);

private:
    static std::mutex mutex;
    static std::string path;
    static const char *levelStr(Level lvl);
};

// RAII 辅助类：析构时自动写入日志
class LogStream {
public:
    LogStream(const Log::Level lvl)
        : lvl(lvl) {}
    ~LogStream() { Log::write(lvl, ss.str()); }

    std::ostringstream &stream() { return ss; }

private:
    Log::Level lvl;
    std::ostringstream ss;
};

/// 流式日志宏：LOG_INFO << "msg" << val;
#define LOG_DEBUG LogStream(Log::debug).stream()
#define LOG_INFO  LogStream(Log::info).stream()
#define LOG_WARN  LogStream(Log::warn).stream()
#define LOG_ERROR LogStream(Log::error).stream()
