#pragma once

#include <chrono>
#include <ctime>
#include <thread>

/// 宿主机的钟：稳时钟计时、超时判断、墙钟转日历、线程休眠。
/// GNSS 时标（CommonTime/WeekSecond 等）在 src/core/TimeStruct.h——那边是数学时间，
/// 这边是「程序运行/调度用的时间」，语义别混。std::chrono 只允许出现在这一个头里，
/// 调用点一律写 `Clock::xxx`（时长类型是 using 进来的，`Clock::seconds(120)` 直接可用）；
/// 超时/到点判断走 `over()` / `due()`，禁再手写 `now - t > d`、`now > deadline`。
namespace Clock {
    /// 稳时钟时间点：测耗时、超时、节流、截止都用它（单调，不受改系统时间影响）。
    using Point = std::chrono::steady_clock::time_point;

    using std::chrono::hours;
    using std::chrono::minutes;
    using std::chrono::seconds;
    using std::chrono::milliseconds;
    using std::chrono::microseconds;
    using std::chrono::duration;
    using std::chrono::duration_cast;

    /// 两个时刻的跨度
    struct Span {
        std::chrono::steady_clock::duration d{};

        [[nodiscard]] long long ms() const { return duration_cast<milliseconds>(d).count(); }

        [[nodiscard]] double sec() const { return duration<double>(d).count(); }

        friend bool operator>(const Span a, const std::chrono::steady_clock::duration b) { return a.d > b; }

        friend bool operator>=(const Span a, const std::chrono::steady_clock::duration b) { return a.d >= b; }

        friend bool operator<(const Span a, const std::chrono::steady_clock::duration b) { return a.d < b; }

        friend bool operator<=(const Span a, const std::chrono::steady_clock::duration b) { return a.d <= b; }
    };

    /// 当前时刻（稳时钟）
    inline Point now() { return std::chrono::steady_clock::now(); }

    /// 距 t 已流逝多久
    inline Span since(const Point t) { return {now() - t}; }

    /// 距 t 是否已满 d
    inline bool over(const Point t, const std::chrono::steady_clock::duration d) {
        return now() - t >= d;
    }

    /// 时刻 t 是否已到
    inline bool due(const Point t) { return now() >= t; }

    /// 从现在起 d 之后
    inline Point after(const std::chrono::steady_clock::duration d) { return now() + d; }

    /// 墙钟当前时刻
    inline std::chrono::system_clock::time_point wallNow() { return std::chrono::system_clock::now(); }

    /// 休眠当前线程
    template<class Rep, class Period>
    void sleep(const duration<Rep, Period> d) {
        std::this_thread::sleep_for(d);
    }

    class Stopwatch {
    public:
        void restart() { mStart = now(); }

        [[nodiscard]] long long ms() const {
            return duration_cast<milliseconds>(now() - mStart).count();
        }

    private:
        Point mStart = now();
    };

    std::tm localTime(std::time_t t);

    std::tm localTime(std::chrono::system_clock::time_point tp);

    inline std::tm localNow() { return localTime(wallNow()); }

    std::tm UTC(std::time_t t);

    std::tm UTC(std::chrono::system_clock::time_point tp);
}
