#include "IniFile.h"

#include <fstream>
#include <string>
#include <map>

namespace AppConfig {

std::string IniFile::trim(const std::string &s) {
    const size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool IniFile::load(const std::string &path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line, cur;
    while (std::getline(f, line)) {
        // 去掉行内注释
        for (const char c : {'#', ';'}) {
            if (const size_t h = line.find(c); h != std::string::npos) line = line.substr(0, h);
        }
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t[0] == '[') {
            if (const size_t e = t.find(']'); e != std::string::npos) cur = t.substr(1, e - 1);
            continue;
        }
        if (const size_t eq = t.find('='); eq != std::string::npos) {
            const std::string k = trim(t.substr(0, eq));
            const std::string v = trim(t.substr(eq + 1));
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

[[nodiscard]] std::string IniFile::get(const std::string &sec, const std::string &key, const std::string &def) const {
    const auto it = sections.find(sec);
    if (it == sections.end()) return def;
    const auto it2 = it->second.find(key);
    if (it2 == it->second.end()) return def;
    return it2->second;
}

[[nodiscard]] int IniFile::getInt(const std::string &sec, const std::string &key, const int def) const {
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    try { return std::stoi(v); } catch (...) { return def; }
}

[[nodiscard]] double IniFile::getD(const std::string &sec, const std::string &key, const double def) const {
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    try { return std::stod(v); } catch (...) { return def; }
}

[[nodiscard]] bool IniFile::getB(const std::string &sec, const std::string &key, const bool def) const {
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    return v == "1" || v == "true" || v == "yes" || v == "on" || v == "TRUE";
}

void IniFile::set(const std::string &sec, const std::string &key, const std::string &val) {
    sections[sec][key] = val;
}

void IniFile::erase(const std::string &sec, const std::string &key) {
    if (const auto it = sections.find(sec); it != sections.end()) it->second.erase(key);
}

} // namespace AppConfig
