#include "StringUtils.h"

#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <charconv>
#include <limits>

namespace Str {
    std::string strip(std::string s) {
        if (s.empty()) return s;
        s.erase(0, s.find_first_not_of(" \t\n\r\f\v"));
        s.erase(s.find_last_not_of(" \t\n\r\f\v") + 1);

        return s;
    }

    bool startsWith(const std::string_view s, const std::string_view p) {
        return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
    }

    namespace {
        bool skipSpace(const char *&cur, const char *end) {
            while (cur < end && (*cur == ' ' || *cur == '\t')) ++cur;
            return cur < end;
        }
    }

    bool nextInt(const char *&cur, const char *end, int &out) {
        const char *p = cur;
        if (!skipSpace(p, end)) return false;
        const auto [ptr, ec] = std::from_chars(p, end, out);
        if (ec != std::errc()) return false;
        cur = ptr;
        return true;
    }

    bool nextDouble(const char *&cur, const char *end, double &out) {
        const char *p = cur;
        if (!skipSpace(p, end)) return false;
        const auto [ptr, ec] = std::from_chars(p, end, out);
        if (ec != std::errc()) return false;
        cur = ptr;
        return true;
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

            return (std::numeric_limits<double>::max)();
        }
    }

    int toInt(const std::string &str) {
        if (str.empty()) {
            return 0;
        }

        std::size_t start = 0;
        while (start < str.size() && std::isspace(static_cast<unsigned char>(str[start]))) {
            start++;
        }

        if (start == str.size()) {
            return 0;
        }

        std::size_t end = str.size() - 1;
        while (end > start && std::isspace(static_cast<unsigned char>(str[end]))) {
            end--;
        }

        const std::string trimmedStr = str.substr(start, end - start + 1);

        try {
            return std::stoi(trimmedStr);
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
        const std::string ls = toLower(s);
        const std::size_t sl = ls.size(), kl = std::strlen(suf);
        return sl >= kl && ls.compare(sl - kl, kl, suf) == 0;
    }

    std::string stripSuffixIC(std::string s, const char *suf) {
        if (const std::size_t kl = std::strlen(suf); s.size() > kl && endsWithIC(s, suf)) s.erase(s.size() - kl);
        return s;
    }


    std::string baseName(const std::string &path) {
        const std::size_t p = path.find_last_of("/\\");
        return p == std::string::npos ? path : path.substr(p + 1);
    }

    std::string sanitizeId(const std::string &s) {
        std::string r;
        r.reserve(s.size());
        for (const char c: s) r += std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' ? c : '_';
        return r;
    }

    std::string stripExt(const std::string &fn) {
        const auto pos = fn.rfind('.');
        return pos != std::string::npos && pos > 0 ? fn.substr(0, pos) : fn;
    }

    std::string fmt4(const double x) {
        char b[32];
        std::snprintf(b, sizeof b, "%.4f", x);
        return b;
    }

    std::string fmt3(const double x) {
        char b[32];
        std::snprintf(b, sizeof b, "%.3f", x);
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

    std::string fileContentHash(const std::string &path) {

        auto fnv = [](const char *b, const std::size_t n, std::uint64_t h) {
            for (std::size_t i = 0; i < n; ++i) {
                h ^= static_cast<std::uint8_t>(b[i]);
                h *= 1099511628211ULL;
            }
            return h;
        };
        std::ifstream f(path, std::ios::binary);
        if (!f) return std::string("ERR:") + path;
        std::uint64_t h = 1469598103934665603ULL;
        f.seekg(0, std::ios::end);
        const std::streamoff sz = f.tellg();
        f.seekg(0);
        h = fnv(reinterpret_cast<const char *>(&sz), sizeof(sz), h);
        constexpr std::streamoff kBlock = 1 << 18;
        std::vector<char> buf(kBlock);
        if (const std::streamoff head = std::min(sz, kBlock); head > 0) {
            f.read(buf.data(), head);
            h = fnv(buf.data(), static_cast<std::size_t>(f.gcount()), h);
        }
        if (sz > 1 << 19) {
            f.seekg(sz - kBlock);
            f.read(buf.data(), kBlock);
            h = fnv(buf.data(), static_cast<std::size_t>(f.gcount()), h);
        }
        char out[20];
        std::snprintf(out, sizeof(out), "%016llx", static_cast<unsigned long long>(h));
        return out;
    }
}
