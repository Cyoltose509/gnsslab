#include "IniFile.h"

#include "StringUtils.h"

#include <fstream>
#include <string>
#include <map>

bool IniFile::load(const std::string &path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line, cur;
    while (std::getline(f, line)) {
        for (const char c : {'#', ';'}) {
            if (const std::size_t h = line.find(c); h != std::string::npos) line = line.substr(0, h);
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
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    try { return std::stoi(v); } catch (...) { return def; }
}

double IniFile::getD(const std::string &sec, const std::string &key, const double def) const {
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    try { return std::stod(v); } catch (...) { return def; }
}

bool IniFile::getB(const std::string &sec, const std::string &key, const bool def) const {
    const std::string v = get(sec, key);
    if (v.empty()) return def;
    const std::string low = Str::toLower(v);
    if (low == "true" || low == "yes" || low == "on") return true;
    if (low == "false" || low == "no" || low == "off") return false;
    try { return std::stoi(v) != 0; } catch (...) { return def; }
}

void IniFile::set(const std::string &sec, const std::string &key, const std::string &val) {
    sections[sec][key] = val;
}

void IniFile::erase(const std::string &sec, const std::string &key) {
    if (const auto it = sections.find(sec); it != sections.end()) it->second.erase(key);
}
