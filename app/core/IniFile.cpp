#include "IniFile.h"

#include "StringUtils.h"

#include <cctype>
#include <charconv>
#include <fstream>
#include <string>
#include <map>

namespace {
    template<class T>
    bool parseNumber(const std::string &s, T &out) {
        if (s.empty()) return false;
        const char *b = s.data();
        const auto [p, ec] = std::from_chars(b, b + s.size(), out);
        return ec == std::errc() && p == b + s.size();
    }
}

bool IniFile::load(const std::string &path) {
    sections.clear();
    std::ifstream f(path);
    if (!f) return false;
    std::string line, cur;
    while (std::getline(f, line)) {
        if (cur.empty() && line.rfind("\xEF\xBB\xBF", 0) == 0) line.erase(0, 3); // 记事本存的 UTF-8 BOM
        // 只有行首或空白之后的 # / ; 才是注释：紧跟等号的值（如以 # 开头的密码）属于值本身。
        for (std::size_t i = 0; i < line.size(); ++i) {
            if ((line[i] != '#' && line[i] != ';') ||
                (i > 0 && !std::isspace(static_cast<unsigned char>(line[i - 1]))))
                continue;
            line.resize(i);
            break;
        }
        const std::string t = Str::strip(line);
        if (t.empty()) continue;
        if (t[0] == '[') {
            if (const std::size_t e = t.find(']'); e != std::string::npos) cur = t.substr(1, e - 1);
            continue;
        }
        if (const std::size_t eq = t.find('='); eq != std::string::npos) {
            const std::string k = Str::strip(t.substr(0, eq));
            const std::string v = Str::strip(t.substr(eq + 1));
            sections[cur][k] = v;
        }
    }
    return true;
}

bool IniFile::save(const std::string &path) const {
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) return false;
    for (const auto &[sec, kvs] : sections) {
        f << '[' << sec << "]\n";
        for (const auto &[k, v] : kvs)
            f << k << '=' << v << '\n';
        f << '\n';
    }
    return true;
}

std::string IniFile::get(const std::string &sec, const std::string &key, const std::string &def) const {
    const auto it = sections.find(sec);
    if (it == sections.end()) return def;
    const auto it2 = it->second.find(key);
    if (it2 == it->second.end()) return def;
    return it2->second;
}

int IniFile::getInt(const std::string &sec, const std::string &key, const int def) const {
    return toInt(get(sec, key), def);
}

double IniFile::getD(const std::string &sec, const std::string &key, const double def) const {
    return toDouble(get(sec, key), def);
}

bool IniFile::getB(const std::string &sec, const std::string &key, const bool def) const {
    return toBool(get(sec, key), def);
}

int IniFile::toInt(const std::string &s, const int def) {
    int v = 0;
    return parseNumber(s, v) ? v : def;
}

double IniFile::toDouble(const std::string &s, const double def) {
    double v = 0;
    return parseNumber(s, v) ? v : def;
}

bool IniFile::toBool(const std::string &s, const bool def) {
    if (s.empty()) return def;
    const std::string low = Str::toLower(s);
    if (low == "true" || low == "yes" || low == "on") return true;
    if (low == "false" || low == "no" || low == "off") return false;
    int n = 0;
    return parseNumber(s, n) ? n != 0 : def;
}

void IniFile::set(const std::string &sec, const std::string &key, const std::string &val) {
    sections[sec][key] = val;
}

void IniFile::erase(const std::string &sec, const std::string &key) {
    if (const auto it = sections.find(sec); it != sections.end()) it->second.erase(key);
}
