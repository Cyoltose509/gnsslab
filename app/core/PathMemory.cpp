#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "core/PathMemory.h"
#include "core/IniFile.h"
#include "Log.h"

#include <algorithm>
#include <cstdio>    // snprintf (setNum)

namespace AppConfig {
    PathMemory &PathMemory::instance() {
        static PathMemory s;
        return s;
    }

    PathMemory::PathMemory() {
        char buf[MAX_PATH] = {};
        const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
        std::string s(buf, n > 0 ? n : 0);
        if (const auto pos = s.find_last_of("\\/"); pos != std::string::npos) s = s.substr(0, pos + 1);
        s += "gnsslab.ini";
        filePath = s;
    }

    namespace {
        // 纯空白视同未设置：用户清空输入框后可能残留空格，若当有效值存下会变成"清不掉"的脏路径。
        bool isBlank(const std::string &s) {
            return s.find_first_not_of(" \t\r\n") == std::string::npos;
        }
    }

    std::string PathMemory::get(const std::string &key, const std::string &def) const {
        IniFile f;
        if (!f.load(filePath)) return def;
        const std::string v = f.get("paths", key, def);
        return isBlank(v) ? def : v; // 脏值（曾存过空白）回落默认，避免把空路径显示给用户
    }

    void PathMemory::set(const std::string &key, const std::string &val) const {
        IniFile f;
        f.load(filePath);
        if (val.empty() || isBlank(val)) f.erase("paths", key);
        else f.set("paths", key, val);
        f.save(filePath);
    }

    double PathMemory::getNum(const std::string &key, const double def, const double lo, const double hi) const {
        const std::string s = get(key);
        if (s.empty()) return def;
        try {
            size_t pos = 0;
            const double v = std::stod(s, &pos);
            if (pos != s.size()) return def; // "12abc" 之类尾缀残留：整体视为无效
            return v >= lo && v <= hi ? v : def;
        } catch (...) {
            return def;
        }
    }

    int PathMemory::getInt(const std::string &key, const int def, const int lo, const int hi) const {
        const std::string s = get(key);
        if (s.empty()) return def;
        try {
            size_t pos = 0;
            const int v = std::stoi(s, &pos);
            if (pos != s.size()) return def;
            return v >= lo && v <= hi ? v : def;
        } catch (...) {
            return def;
        }
    }

    void PathMemory::setNum(const std::string &key, const double v) const {
        char b[32];
        std::snprintf(b, sizeof(b), "%.12g", v);
        set(key, b);
    }

    std::vector<std::string> PathMemory::getRecent(const std::string &group) const {
        std::vector<std::string> v;
        IniFile f;
        if (!f.load(filePath)) return v;
        const std::string sec = "recent_" + group;
        // 索引从 0 连续排列，遇空即止（addRecent 保证不留空洞）
        for (int i = 0;; ++i) {
            const std::string e = f.get(sec, std::to_string(i));
            if (e.empty()) break;
            v.push_back(e);
        }
        return v;
    }

    void PathMemory::addRecent(const std::string &group, const std::string &entry, const int max) const {
        if (entry.empty()) return;
        std::vector<std::string> v = getRecent(group);
        // 去重（精确匹配），再把新项放到最前
        v.erase(std::remove_if(v.begin(), v.end(),
                               [&](const std::string &s) { return s == entry; }),
                v.end());
        v.insert(v.begin(), entry);
        if (static_cast<int>(v.size()) > max) v.resize(max);

        IniFile f;
        f.load(filePath);
        const std::string sec = "recent_" + group;
        for (int i = 0; i < static_cast<int>(v.size()); ++i)
            f.set(sec, std::to_string(i), v[i]);
        // 删除可能残留的旧索引（收缩时）
        for (int i = static_cast<int>(v.size());; ++i) {
            const std::string k = std::to_string(i);
            if (f.get(sec, k).empty()) break;
            f.erase(sec, k);
        }
        f.save(filePath);
    }
}
