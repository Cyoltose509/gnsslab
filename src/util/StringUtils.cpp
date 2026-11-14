#include "StringUtils.h"

#include <windows.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace Str {
    std::string strip(std::string s) {
        if (s.empty()) return s;
        s.erase(0, s.find_first_not_of(" \t\n\r\f\v"));
        s.erase(s.find_last_not_of(" \t\n\r\f\v") + 1);

        return s;
    }

    std::string leadingAlpha(const std::string_view s) {        std::size_t n = 0;
        while (n < s.size() && std::isalpha(static_cast<unsigned char>(s[n])) != 0) ++n;
        return std::string(s.substr(0, n));
    }



    int scanDoubles(const char *s, double *out, const int maxN) {
        int cnt = 0;
        while (s && *s && cnt < maxN) {
            while (*s && *s != '-' && (*s < '0' || *s > '9')) s++;
            if (!*s) break;
            const auto [ptr, ec] = std::from_chars(s, s + std::strlen(s), out[cnt]);
            if (ec != std::errc()) {
                s++;
                continue;
            }
            s = ptr;
            cnt++;
        }
        return cnt;
    }


    int scanMeters(const char *s, double *out, const int maxN) {
        const int n = scanDoubles(s, out, maxN);
        if (n == 0 || s == nullptr) return n;
        const std::string_view text(s);
        double scale = 1.0;
        if (text.find("mm") != std::string_view::npos || text.find("MM") != std::string_view::npos ||
            text.find("毫米") != std::string_view::npos)
            scale = 1.0E-3;
        else if (text.find("cm") != std::string_view::npos || text.find("CM") != std::string_view::npos ||
                 text.find("厘米") != std::string_view::npos)
            scale = 1.0E-2;
        if (scale != 1.0)
            for (int i = 0; i < n; i++) out[i] *= scale;
        return n;
    }

    std::vector<std::string> split(const std::string_view s, const char sep, const bool keepEmpty) {
        std::vector<std::string> out;
        std::size_t start = 0;
        for (;;) {
            const std::size_t p = s.find(sep, start);
            if (p == std::string_view::npos) break;
            if (keepEmpty || p != start) out.emplace_back(s.substr(start, p - start));
            start = p + 1;
        }
        if (const std::string_view tail = s.substr(start); keepEmpty || !tail.empty()) out.emplace_back(tail);
        return out;
    }

    std::string join(const std::vector<std::string> &v, const char sep) {
        std::string s;
        for (const auto &x: v) {
            if (!s.empty()) s += sep;
            s += x;
        }
        return s;
    }

    std::string &stripTrailing(std::string &line) {
        if (const std::size_t end = line.find_last_not_of(" \t\n\r\v\f"); end != std::string::npos) {
            line.resize(end + 1);
        } else {
            line.clear();
        }
        return line;
    }

    std::string sub(const std::string &str, const std::size_t pos, const std::size_t len) {
        if (pos >= str.size()) return {};
        return str.substr(pos, (std::min)(len, str.size() - pos));
    }

    double toDouble(const std::string &str, const double defaultValue) {
        if (str.empty() || str.find_first_not_of(' ') == std::string::npos) {
            return defaultValue;
        }

        try {
            return std::stod(str);
        } catch ([[maybe_unused]] const std::invalid_argument &e) {
            return defaultValue;
        } catch ([[maybe_unused]] const std::out_of_range &e) {
            return defaultValue; // 溢出与解析失败同责：不回 DBL_MAX 污染解算
        }
    }

    int toInt(const std::string &str) {
        const std::string t = strip(str);
        if (t.empty()) return 0;
        try {
            return std::stoi(t);
        } catch ([[maybe_unused]] const std::invalid_argument &e) {
            return 0;
        } catch ([[maybe_unused]] const std::out_of_range &e) {
            return 0;
        }
    }


    std::string toLower(std::string s) {
        for (char &c: s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    }


    std::string toUpper(std::string s) {
        for (char &c: s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        return s;
    }


    bool endsWith(const std::string &s, const char *suf) {
        const std::string t(suf);
        return s.size() >= t.size() && s.compare(s.size() - t.size(), t.size(), t) == 0;
    }


    bool endsWithIC(const std::string &s, const char *suf) {
        const std::size_t kl = std::strlen(suf);
        if (s.size() < kl) return false;
        for (std::size_t i = 0; i < kl; ++i) {
            if (std::tolower(static_cast<unsigned char>(s[s.size() - kl + i]))
                    != std::tolower(static_cast<unsigned char>(suf[i]))) return false;
        }
        return true;
    }

    std::string stripSuffixIC(std::string s, const char *suf) {
        if (const std::size_t kl = std::strlen(suf); s.size() > kl && endsWithIC(s, suf)) s.erase(s.size() - kl);
        return s;
    }


    std::string sanitizeId(const std::string &s) {
        std::string r;
        r.reserve(s.size());
        for (const char c: s) r += std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' ? c : '_';
        return r;
    }

    std::string fmt4(const double x) {
        char b[32];
        std::snprintf(b, sizeof b, "%.4f", x);
        return b;
    }

    std::string fmtG(const double x) {
        char b[32];
        std::snprintf(b, sizeof b, "%.12g", x);
        return b;
    }

    std::wstring UTF8ToWide(const std::string &s) {
        if (s.empty()) return {};
        const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
        std::wstring w(n > 0 ? n - 1 : 0, L'\0');
        if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
        return w;
    }

    std::string wideToUTF8(const std::wstring &w) {
        if (w.empty()) return {};
        const int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
        std::string s(n > 0 ? n - 1 : 0, '\0');
        if (n > 0) WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
        return s;
    }

    std::string wideToACP(const std::wstring &w) {
        if (w.empty()) return {};
        const int n = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
        std::string s(n > 0 ? n - 1 : 0, '\0');
        if (n > 0) WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, s.data(), n, nullptr, nullptr);
        return s;
    }

    std::wstring ACPToWide(const std::string &s) {
        if (s.empty()) return {};
        const int n = MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, nullptr, 0);
        std::wstring w(n > 0 ? n - 1 : 0, L'\0');
        if (n > 0) MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, &w[0], n);
        return w;
    }
}
