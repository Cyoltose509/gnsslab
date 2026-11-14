#include <windows.h>
#include "core/PathMemory.h"
#include "StringUtils.h"

#include <charconv>
#include <vector>

namespace AppConfig {
    PathMemory &PathMemory::instance() {
        static PathMemory s;
        return s;
    }

    PathMemory::PathMemory() {
        // MAX_PATH 只够常规安装位置；路径更长时按实际长度扩容，别让截断悄悄改写配置文件位置。
        std::vector<char> buf(MAX_PATH);
        while (true) {
            const DWORD n = GetModuleFileNameA(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
            if (n < static_cast<DWORD>(buf.size())) break;
            buf.resize(buf.size() * 2);
        }
        std::string s(buf.data());
        if (const auto pos = s.find_last_of("\\/"); pos != std::string::npos) s = s.substr(0, pos + 1);
        s += "gnsslab.ini";
        filePath = s;
    }

    namespace {
        bool isBlank(const std::string &s) {
            return Str::strip(s).empty();
        }

        template<class T>
        bool parseExact(const std::string &s, T &out) {
            const char *b = s.data();
            const auto [p, ec] = std::from_chars(b, b + s.size(), out);
            return ec == std::errc() && p == b + s.size();
        }
    }

    IniFile &PathMemory::file() const {
        if (!loaded) {
            cache.load(filePath);
            loaded = true;
        }
        return cache;
    }

    void PathMemory::flush() const {
        if (!dirty) return;
        if (cache.save(filePath)) dirty = false;
    }

    std::string PathMemory::get(const std::string &key, const std::string &def) const {
        const std::string v = file().get("paths", key, def);
        return isBlank(v) ? def : v;
    }

    void PathMemory::set(const std::string &key, const std::string &val) const {
        IniFile &f = file();
        const std::string cur = f.get("paths", key);
        if (val.empty() || isBlank(val)) {
            if (cur.empty()) return;
            f.erase("paths", key);
        } else {
            if (cur == val) return;
            f.set("paths", key, val);
        }
        dirty = true;
    }

    double PathMemory::getNum(const std::string &key, const double def, const double lo, const double hi) const {
        double v = 0;
        if (!parseExact(get(key), v)) return def;
        return v >= lo && v <= hi ? v : def;
    }

    int PathMemory::getInt(const std::string &key, const int def, const int lo, const int hi) const {
        int v = 0;
        if (!parseExact(get(key), v)) return def;
        return v >= lo && v <= hi ? v : def;
    }

    void PathMemory::setNum(const std::string &key, const double v) const {
        set(key, Str::fmtG(v)); // %.12g
    }

    void PathMemory::setInt(const std::string &key, const int v) const {
        set(key, std::to_string(v));
    }
}
