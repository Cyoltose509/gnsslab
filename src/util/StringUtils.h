#pragma once

#include <charconv>
#include <string>
#include <string_view>
#include <vector>

namespace Str {
    std::string strip(std::string s);

    bool startsWith(std::string_view s, std::string_view p);

    std::string leadingAlpha(std::string_view s);

    template<class T>
    bool next(const char *&cur, const char *end, T &out) {
        while (cur < end && (*cur == ' ' || *cur == '\t')) ++cur;
        if (cur >= end) return false;
        const auto [ptr, ec] = std::from_chars(cur, end, out);
        if (ec != std::errc()) return false;
        cur = ptr;
        return true;
    }

    int scanDoubles(const char *s, double *out, int maxN);

    /// 同 `scanDoubles`，但认文本里的 `mm`/`cm`/`毫米`/`厘米` 并把结果统一换成 m（没写单位按 m）。
    /// 手册、SINEX 摘抄、剪贴板三种来源单位混着来，换算只在这里做一次。
    int scanMeters(const char *s, double *out, int maxN);

    std::vector<std::string> split(std::string_view s, char sep, bool keepEmpty = true);

    std::string join(const std::vector<std::string> &v, char sep);

    std::string &stripTrailing(std::string &line);

    std::string sub(const std::string &str, std::size_t pos, std::size_t len);

    double toDouble(const std::string &str, double defaultValue = 0.0);

    int toInt(const std::string &str);

    std::string toLower(std::string s);

    std::string toUpper(std::string s);

    bool endsWith(const std::string &s, const char *suf);

    bool endsWithIC(const std::string &s, const char *suf);

    std::string stripSuffixIC(std::string s, const char *suf);

    std::string sanitizeId(const std::string &s);

    std::string fmt4(double x);

    std::string fmtG(double x);

    std::wstring UTF8ToWide(const std::string &s);

    std::string wideToUTF8(const std::wstring &w);

    std::string wideToACP(const std::wstring &w);

    std::wstring ACPToWide(const std::string &s);
}
