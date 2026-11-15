#include "FileUtils.h"
#include "i18n/I18n.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include "i18n/I18n.h"

namespace File {
    bool safeWrite(const std::filesystem::path &path,
                   const std::function<void(std::ostream &)> &write,
                   std::string &error, const bool binary) {
        std::ofstream out(path, binary ? std::ios::out | std::ios::binary : std::ios::out);
        if (!out) {
            error = I18n::tr("util.file.write_no_dir", path.string());
            return false;
        }
        write(out);
        out.flush();
        if (!out) {
            error = I18n::tr("util.file.write_busy", path.string());
            return false;
        }
        return true;
    }

    bool forEachLine(const std::string &path, const std::function<void(std::string_view)> &fn) {
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

    std::string baseName(const std::string &path) {
        const std::size_t p = path.find_last_of("/\\");
        return p == std::string::npos ? path : path.substr(p + 1);
    }

    std::string stripExt(const std::string &fn) {
        const auto pos = fn.rfind('.');
        return pos != std::string::npos && pos > 0 ? fn.substr(0, pos) : fn;
    }

    std::string contentHash(const std::string &path) {
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
