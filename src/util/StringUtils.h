#pragma once

#include <string>
#include <stdexcept>
#include <limits>
#include <fstream>
#include <vector>
#include <cctype>
#include <cstring>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cstdio>   // snprintf (fmt3/fmt4)

using namespace std;

inline string strip(std::string s) {
    // 如果字符串为空，则无需处理
    if (s.empty()) return s;

    // 移除头部的空白字符
    s.erase(0, s.find_first_not_of(" \t\n\r\f\v"));

    // 移除尾部的空白字符
    s.erase(s.find_last_not_of(" \t\n\r\f\v") + 1);

    return s;
}

inline std::string &stripTrailing(std::string &line) {
    if (const size_t end = line.find_last_not_of(" \t\n\r\v\f"); end != std::string::npos) {
        line.resize(end + 1); // 修改字符串的长度，去掉后面的空白字符
    } else {
        line.clear(); // 如果全是空白字符，清空字符串
    }
    return line; // 返回修改后的字符串的引用
}

/// 安全的 substr：起始越界返回空串，长度超界自动截断
inline std::string safeSubstr(const std::string &str, const size_t pos, const size_t len) {
    if (pos >= str.size()) return {};
    return str.substr(pos, (std::min)(len, str.size() - pos));
}

inline double safeStod(const std::string &str, const double defaultValue = 0.0) {
    if (str.empty() || str.find_first_not_of(' ') == std::string::npos) {
        // 字符串为空或只包含空白字符
        return defaultValue;
    }

    try {
        return std::stod(str);
    } catch ([[maybe_unused]] const std::invalid_argument &e) {
        // 字符串不是有效的浮点数表示
        return defaultValue;
    } catch ([[maybe_unused]] const std::out_of_range &e) {
        // 转换后的数值超出了 double 类型可表示的范围
        return (std::numeric_limits<double>::max)();
    }
}

inline int safeStoi(const std::string &str) {
    if (str.empty()) {
        return 0; // 如果字符串为空，直接返回 0
    }

    size_t start = 0;
    while (start < str.size() && std::isspace(static_cast<unsigned char>(str[start]))) {
        start++; // 跳过开头的空白字符
    }

    if (start == str.size()) {
        return 0; // 如果全是空白字符，返回 0
    }

    size_t end = str.size() - 1;
    while (end > start && std::isspace(static_cast<unsigned char>(str[end]))) {
        end--; // 跳过结尾的空白字符
    }

    const std::string trimmedStr = str.substr(start, end - start + 1);

    try {
        return std::stoi(trimmedStr);
    } catch ([[maybe_unused]] const std::invalid_argument &e) {
        return 0; // 如果转换失败，返回 0
    } catch ([[maybe_unused]] const std::out_of_range &e) {
        return 0; // 如果整数超出范围，返回 0
    }
}

inline std::wstring utf8ToWide(const std::string &s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w(n > 0 ? n - 1 : 0, L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
    return w;
}

inline std::string wideToUtf8(const std::wstring &w) {
    if (w.empty()) return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, '\0');
    if (n > 0) WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
    return s;
}

inline std::string sanitizeId(const std::string &s) {
    std::string r;
    r.reserve(s.size());
    for (const char c: s) r += std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' ? c : '_';
    return r;
}

inline std::string stripExt(const std::string &fn) {
    const auto pos = fn.rfind('.');
    return pos != std::string::npos && pos > 0 ? fn.substr(0, pos) : fn;
}

/// 转小写（逐字符，ASCII 安全：先转 unsigned char 再进 std::tolower，避免负值 UB）
inline std::string toLower(std::string s) {
    for (char &c: s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

/// 大小写不敏感的后缀判断（suf 须传小写形式，如 ".asc"）
inline bool endsWithIC(const std::string &s, const char *suf) {
    const std::string ls = toLower(s);
    const size_t sl = ls.size(), kl = std::strlen(suf);
    return sl >= kl && ls.compare(sl - kl, kl, suf) == 0;
}

static std::string wideToAcp(const std::wstring &w) {
    if (w.empty()) return {};
    const int n = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, '\0');
    if (n > 0) WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
    return s;
}

inline std::wstring acpToWide(const std::string &s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, nullptr, 0);
    std::wstring w(n > 0 ? n - 1 : 0, L'\0');
    if (n > 0) MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, &w[0], n);
    return w;
}

inline std::string fmt4(const double x) {
    char b[32];
    snprintf(b, sizeof b, "%.4f", x);
    return b;
}

inline std::string fmt3(const double x) {
    char b[32];
    snprintf(b, sizeof b, "%.3f", x);
    return b;
}

/// %.12g：浮点的完整有效数字往返（ini/CSV 持久化用；fmt3/fmt4 是定长显示，
/// std::to_string(double) 只有 6 位小数会截断，均不适用于存读回）
inline std::string fmtG(const double x) {
    char b[32];
    snprintf(b, sizeof b, "%.12g", x);
    return b;
}

/// 逐行回调；文件打不开返回 false（与 ifstream 构造失败同义）
template<class F>
bool forEachLine(const std::string &path, F &&fn) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;

    constexpr std::streamsize kChunk = 1 << 20; // 1 MB
    std::vector<char> buf(kChunk);
    std::string carry; // 跨块残行（被截断的那一行最多一次小拷贝）

    for (;;) {
        in.read(buf.data(), kChunk);
        const std::streamsize got = in.gcount();
        if (got <= 0) break;

        const char *p = buf.data();
        const char *end = p + got;
        const char *sol = p; // 当前行起点
        for (const char *q = p; q != end; ++q) {
            if (*q != '\n') continue;
            const char *le = q;
            if (le > sol && le[-1] == '\r') --le; // 去掉 CR
            const std::string_view sv(sol, static_cast<size_t>(le - sol));
            buf[static_cast<size_t>(le - p)] = '\0'; // 行尾字节（'\r' 或 '\n'）之后不再用
            if (carry.empty()) {
                fn(sv);
            } else {
                carry.append(sv);
                fn(std::string_view(carry));
                carry.clear();
            }
            sol = q + 1;
        }
        if (sol < end) carry.append(sol, static_cast<size_t>(end - sol));
        if (got < kChunk) break;
    }
    if (!carry.empty()) fn(std::string_view(carry)); // 末行无换行符
    return true;
}
