#include "Clock.h"

namespace Clock {
    std::tm localTime(const std::time_t t) {
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        return tm;
    }

    std::tm localTime(const std::chrono::system_clock::time_point tp) {
        return localTime(std::chrono::system_clock::to_time_t(tp));
    }

    std::tm UTC(const std::time_t t) {
        std::tm tm{};
#if defined(_WIN32)
        gmtime_s(&tm, &t);
#else
        gmtime_r(&t, &tm);
#endif
        return tm;
    }

    std::tm UTC(const std::chrono::system_clock::time_point tp) {
        return UTC(std::chrono::system_clock::to_time_t(tp));
    }
}
