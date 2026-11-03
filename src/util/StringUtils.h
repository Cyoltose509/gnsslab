#pragma once

#include <string>
#include <string_view>
#include <stdexcept>
#include <fstream>
#include <vector>

namespace Str {
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
                const std::string_view sv(sol, static_cast<std::size_t>(le - sol));
                buf[static_cast<std::size_t>(le - p)] = '\0'; // 行尾字节（'\r' 或 '\n'）之后不再用
                if (carry.empty()) {
                    fn(sv);
                } else {
                    carry.append(sv);
                    fn(std::string_view(carry));
                    carry.clear();
                }
                sol = q + 1;
            }
            if (sol < end) carry.append(sol, static_cast<std::size_t>(end - sol));
            if (got < kChunk) break;
        }
        if (!carry.empty()) fn(std::string_view(carry)); // 末行无换行符
        return true;
    }

    std::string strip(std::string s);

    bool startsWith(std::string_view s, std::string_view p);

    bool nextInt(const char *&cur, const char *end, int &out);

    bool nextDouble(const char *&cur, const char *end, double &out);

    // 从混有任意文本的串里依次扫出至多 maxN 个浮点数（跳过非数字字符），返回实读个数。
    int scanDoubles(const char *s, double *out, int maxN);

    std::vector<std::string> split(std::string_view s, char sep, bool keepEmpty = true);

    std::string &stripTrailing(std::string &line);

    std::string sub(const std::string &str, std::size_t pos, std::size_t len);

    double toDouble(const std::string &str, double defaultValue = 0.0);

    int toInt(const std::string &str);

    std::string toLower(std::string s);

    std::string toUpper(std::string s);

    bool endsWith(const std::string &s, const char *suf);

    bool endsWithIC(const std::string &s, const char *suf);

    std::string stripSuffixIC(std::string s, const char *suf);

    std::string baseName(const std::string &path);

    std::string sanitizeId(const std::string &s);

    std::string stripExt(const std::string &fn);

    std::string fmt4(double x);

    std::string fmt3(double x);

    std::string fmtG(double x);

    std::wstring UTF8ToWide(const std::string &s);

    std::string wideToUTF8(const std::wstring &w);

    std::string wideToACP(const std::wstring &w);

    std::wstring ACPToWide(const std::string &s);

    std::string fileContentHash(const std::string &path);
}
