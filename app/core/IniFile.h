#pragma once
#include <string>
#include <map>

namespace AppConfig {

struct IniFile {
    std::map<std::string, std::map<std::string, std::string>> sections;


    static std::string trim(const std::string &s);


    bool load(const std::string &path);


    bool save(const std::string &path) const;


    [[nodiscard]] std::string get(const std::string &sec, const std::string &key, const std::string &def = "") const;
    [[nodiscard]] int getInt(const std::string &sec, const std::string &key, const int def = 0) const;
    [[nodiscard]] double getD(const std::string &sec, const std::string &key, const double def = 0.0) const;
    [[nodiscard]] bool getB(const std::string &sec, const std::string &key, const bool def = false) const;


    void set(const std::string &sec, const std::string &key, const std::string &val);

    void erase(const std::string &sec, const std::string &key);
};

}
