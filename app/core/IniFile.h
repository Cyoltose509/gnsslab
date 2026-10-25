#pragma once
#include <string>
#include <map>

namespace AppConfig {

struct IniFile {
    std::map<std::string, std::map<std::string, std::string>> sections;

    // 去首尾空白（空格 / 制表符 / 回车 / 换行）。
    static std::string trim(const std::string &s);

    // 从文件载入；任何打开或解析失败返回 false（保留已解析的部分）。
    bool load(const std::string &path);

    // 写回文件（截断重写）。返回是否成功打开。
    bool save(const std::string &path) const;

    // 取键；缺失返回 def。
    [[nodiscard]] std::string get(const std::string &sec, const std::string &key, const std::string &def = "") const;
    [[nodiscard]] int getInt(const std::string &sec, const std::string &key, const int def = 0) const;
    [[nodiscard]] double getD(const std::string &sec, const std::string &key, const double def = 0.0) const;
    [[nodiscard]] bool getB(const std::string &sec, const std::string &key, const bool def = false) const;

    // 设键（不存在则插入）。
    void set(const std::string &sec, const std::string &key, const std::string &val);
    // 删一个键（用于 addRecent 收缩时清掉残留的旧索引）。
    void erase(const std::string &sec, const std::string &key);
};

} // namespace AppConfig
