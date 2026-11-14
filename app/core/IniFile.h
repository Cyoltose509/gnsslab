#pragma once
#include <string>
#include <map>

inline constexpr char kIniListSep = '|';

struct IniFile {
    std::map<std::string, std::map<std::string, std::string> > sections;


    bool load(const std::string &path);


    bool save(const std::string &path) const;


    [[nodiscard]] std::string get(const std::string &sec, const std::string &key, const std::string &def = "") const;

    [[nodiscard]] int getInt(const std::string &sec, const std::string &key, int def = 0) const;

    [[nodiscard]] double getD(const std::string &sec, const std::string &key, double def = 0.0) const;

    [[nodiscard]] bool getB(const std::string &sec, const std::string &key, bool def = false) const;

    /// 文本 → 数值/布尔，ini 键值与命令行覆盖值共用这一套转换。
    static int toInt(const std::string &s, int def);
    static double toDouble(const std::string &s, double def);
    static bool toBool(const std::string &s, bool def);


    void set(const std::string &sec, const std::string &key, const std::string &val);

    void erase(const std::string &sec, const std::string &key);
};
